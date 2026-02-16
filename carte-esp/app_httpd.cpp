#include <esp_http_server.h>
#include <esp_timer.h>
#include <esp_camera.h>
#include <img_converters.h>

#include "globals.hpp"

#define SEND_CHUNK_OR_BREAK(x) \
  if ((x) != ESP_OK) { \
    res = ESP_FAIL; \
    break; \
  }

#define PWMA_PIN 12
#define PWMB_PIN 13

#define PWMA 0
#define PWMB 1

int position_servo = 90;

int AIN1_val = 0;
int AIN2_val = 0;

int BIN1_val = 0;
int BIN2_val = 0;

int speed = 100;

volatile int ModMove = 0;
volatile bool robot_fwd_val = false;

void sendToMega();

void robot_setup();
void robot_stop();
void robot_fwd();
void robot_back();
void robot_left();
void robot_right();

void camera_left();
void camera_right();
void camera_center();

void robot_setup() {
  const int pwmFreq = 20000;
  const int pwmRes = 8;

  // Configuration d'abord
  ledcSetup(0, pwmFreq, pwmRes);
  ledcAttachPin(PWMA_PIN, PWMA);

  ledcSetup(1, pwmFreq, pwmRes);
  ledcAttachPin(PWMB_PIN, PWMB);

  pinMode(33, OUTPUT);

  robot_stop();
  sendToMega();
}

// Motor Control Functions
void robot_stop() {
  AIN1_val = 0;
  AIN2_val = 0;

  BIN1_val = 0;
  BIN2_val = 0;

  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);

  if (speed >= 100) {
    speed = 100;
  }
  robot_fwd_val = false;

  sendToMega();
}

void robot_fwd() {
  if (ModMove == 1 && DistFront <= 20) {
    robot_stop();
    return;
  }

  robot_fwd_val = true;

  AIN1_val = 0;
  AIN2_val = 1;

  BIN1_val = 0;
  BIN2_val = 1;

  ledcWrite(PWMA, speed);
  ledcWrite(PWMB, speed);

  sendToMega();
}

void robot_back() {
  AIN1_val = 1;
  AIN2_val = 0;

  BIN1_val = 1;
  BIN2_val = 0;

  ledcWrite(PWMA, speed);
  ledcWrite(PWMB, speed);

  sendToMega();
}

void robot_right() {
  AIN1_val = 0;
  AIN2_val = 1;

  BIN1_val = 1;
  BIN2_val = 0;

  ledcWrite(PWMA, speed);
  ledcWrite(PWMB, speed);

  sendToMega();
}

void robot_left() {
  AIN1_val = 1;
  AIN2_val = 0;

  BIN1_val = 0;
  BIN2_val = 1;

  ledcWrite(PWMA, speed);
  ledcWrite(PWMB, speed);

  sendToMega();
}

void camera_left() {
  if (position_servo < 180) {
    position_servo += 10;
  }
  sendToMega();
}

void camera_right() {
  if (position_servo > 0) {
    position_servo -= 10;
  }
  sendToMega();
}

void camera_center() {
  position_servo = 90;
  sendToMega();
}

void sendToMega() {
  static int last_AIN1 = -1;
  static int last_AIN2 = -1;
  static int last_BIN1 = -1;
  static int last_BIN2 = -1;
  static int last_servo = -1;

  if (AIN1_val == last_AIN1 && AIN2_val == last_AIN2 && BIN1_val == last_BIN1 && BIN2_val == last_BIN2 && position_servo == last_servo) {
    return;  // rien à envoyer
  }

  last_AIN1 = AIN1_val;
  last_AIN2 = AIN2_val;
  last_BIN1 = BIN1_val;
  last_BIN2 = BIN2_val;
  last_servo = position_servo;

  Serial.printf("%d,%d,%d,%d,%d\n", AIN1_val, AIN2_val, BIN1_val, BIN2_val, position_servo);
}

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t server_rover = NULL;

static esp_err_t capture_handler(httpd_req_t *req) {
  // Capture d'une frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    // Serial.println("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  // On force le type JPEG
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");

  // Envoi direct du buffer JPEG
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);

  // Libération de la frame
  esp_camera_fb_return(fb);

  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;
  static char part_buf[128];  // réutilisable
  httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 33 / portTICK_PERIOD_MS;  // 30 FPS ~33ms

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(1);  // petite pause si capture échoue
      continue;
    }

    // Envoi MJPEG
    if (httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY)) != ESP_OK) {
      esp_camera_fb_return(fb);
      res = ESP_FAIL;
      break;
    }

    int hlen = sprintf(part_buf, "--boundary\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
    if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK || httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len) != ESP_OK) {
      esp_camera_fb_return(fb);
      res = ESP_FAIL;
      break;
    }

    esp_camera_fb_return(fb);

    // cadence stable 30 FPS
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }

  return res;
}

static void add_cors_headers(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
  httpd_resp_set_hdr(req, "Access-Control-Max-Age", "86400");
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  add_cors_headers(req);
  char *buf;
  size_t buf_len;
  char variable[32] = {
    0,
  };
  char value[32] = {
    0,
  };

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK && httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  int val = atoi(value);
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;

  if (!strcmp(variable, "framesize")) {
    if (s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
  } else if (!strcmp(variable, "quality")) res = s->set_quality(s, val);
  else if (!strcmp(variable, "contrast")) res = s->set_contrast(s, val);
  else if (!strcmp(variable, "brightness")) res = s->set_brightness(s, val);
  else if (!strcmp(variable, "saturation")) res = s->set_saturation(s, val);
  else if (!strcmp(variable, "gainceiling")) res = s->set_gainceiling(s, (gainceiling_t)val);
  else if (!strcmp(variable, "colorbar")) res = s->set_colorbar(s, val);
  else if (!strcmp(variable, "awb")) res = s->set_whitebal(s, val);
  else if (!strcmp(variable, "agc")) res = s->set_gain_ctrl(s, val);
  else if (!strcmp(variable, "aec")) res = s->set_exposure_ctrl(s, val);
  else if (!strcmp(variable, "hmirror")) res = s->set_hmirror(s, val);
  else if (!strcmp(variable, "vflip")) res = s->set_vflip(s, val);
  else if (!strcmp(variable, "awb_gain")) res = s->set_awb_gain(s, val);
  else if (!strcmp(variable, "agc_gain")) res = s->set_agc_gain(s, val);
  else if (!strcmp(variable, "aec_value")) res = s->set_aec_value(s, val);
  else if (!strcmp(variable, "aec2")) res = s->set_aec2(s, val);
  else if (!strcmp(variable, "dcw")) res = s->set_dcw(s, val);
  else if (!strcmp(variable, "bpc")) res = s->set_bpc(s, val);
  else if (!strcmp(variable, "wpc")) res = s->set_wpc(s, val);
  else if (!strcmp(variable, "raw_gma")) res = s->set_raw_gma(s, val);
  else if (!strcmp(variable, "lenc")) res = s->set_lenc(s, val);
  else if (!strcmp(variable, "special_effect")) res = s->set_special_effect(s, val);
  else if (!strcmp(variable, "wb_mode")) res = s->set_wb_mode(s, val);
  else if (!strcmp(variable, "ae_level")) res = s->set_ae_level(s, val);
  else {
    res = -1;
  }

  if (res) {
    return httpd_resp_send_500(req);
  }

  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req) {
  add_cors_headers(req);
  char json_response[512];  // suffit si pas trop de champs
  sensor_t *s = esp_camera_sensor_get();
  int len = snprintf(json_response, sizeof(json_response),
                     "{\"framesize\":%u,\"quality\":%u,\"brightness\":%d,\"contrast\":%d,"
                     "\"saturation\":%d,\"special_effect\":%u,\"wb_mode\":%u,\"awb\":%u,"
                     "\"awb_gain\":%u,\"aec\":%u,\"aec2\":%u,\"ae_level\":%d,\"aec_value\":%u,"
                     "\"agc\":%u,\"agc_gain\":%u,\"gainceiling\":%u,\"bpc\":%u,\"wpc\":%u,"
                     "\"raw_gma\":%u,\"lenc\":%u,\"hmirror\":%u,\"dcw\":%u,\"colorbar\":%u}",
                     s->status.framesize, s->status.quality, s->status.brightness, s->status.contrast,
                     s->status.saturation, s->status.special_effect, s->status.wb_mode, s->status.awb,
                     s->status.awb_gain, s->status.aec, s->status.aec2, s->status.ae_level, s->status.aec_value,
                     s->status.agc, s->status.agc_gain, s->status.gainceiling, s->status.bpc, s->status.wpc,
                     s->status.raw_gma, s->status.lenc, s->status.hmirror, s->status.dcw, s->status.colorbar);

  if (len < 0) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json_response, len);
}

static esp_err_t cors_options_handler(httpd_req_t *req) {
  add_cors_headers(req);
  return httpd_resp_send(req, NULL, 0);  // réponse vide
}

static esp_err_t index_handler(httpd_req_t *req) {
  add_cors_headers(req);

  const char page[] PROGMEM = R"rawliteral(Utilisez la page index.html en serveur local)rawliteral";

  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t go_handler(httpd_req_t *req) {
  add_cors_headers(req);
  robot_fwd();
  // Serial.println("Go");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t back_handler(httpd_req_t *req) {
  add_cors_headers(req);
  robot_back();
  // Serial.println("Back");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t left_handler(httpd_req_t *req) {
  add_cors_headers(req);
  robot_left();
  // Serial.println("Left");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}
static esp_err_t right_handler(httpd_req_t *req) {
  add_cors_headers(req);
  robot_right();
  // Serial.println("Right");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t stop_handler(httpd_req_t *req) {
  add_cors_headers(req);
  robot_stop();
  // Serial.println("Stop");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t ledon_handler(httpd_req_t *req) {
  add_cors_headers(req);
  digitalWrite(gpLed, HIGH);
  // Serial.println("LED ON");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t ledoff_handler(httpd_req_t *req) {
  add_cors_headers(req);
  digitalWrite(gpLed, LOW);
  // Serial.println("LED OFF");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t cam_left_handler(httpd_req_t *req) {
  add_cors_headers(req);
  camera_left();
  // Serial.println("CAM LEFT");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t cam_right_handler(httpd_req_t *req) {
  add_cors_headers(req);
  camera_right();
  // Serial.println("CAM RIGHT");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t cam_center_handler(httpd_req_t *req) {
  add_cors_headers(req);
  camera_center();
  // Serial.println("CAM CENTER");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t move_standart_handler(httpd_req_t *req) {
  add_cors_headers(req);
  ModMove = 0;
  // Serial.println("MOD MOVE 0");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t move_stop_obstacle_handler(httpd_req_t *req) {
  add_cors_headers(req);
  ModMove = 1;
  // Serial.println("MOD MOVE 1");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t up_speed_handler(httpd_req_t *req) {
  add_cors_headers(req);
  if (speed < 255) { speed += 5; }
  // Serial.println("Speep up");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t down_speed_handler(httpd_req_t *req) {
  add_cors_headers(req);
  if (speed > 0) { speed -= 5; }
  // Serial.println("Speed down");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

// Échappe " et \ pour JSON, modifie le buffer output
void escape_json(const char *input, char *output, size_t outSize) {
  size_t j = 0;
  for (size_t i = 0; input[i] != '\0' && j < outSize - 1; i++) {
    char c = input[i];
    if (c == '\"' || c == '\\') {
      if (j + 2 >= outSize - 1) break;  // pas assez de place
      output[j++] = '\\';
      output[j++] = c;
    } else if (c == '\n') {
      if (j + 2 >= outSize - 1) break;
      output[j++] = '\\';
      output[j++] = 'n';
    } else if (c == '\r') {
      if (j + 2 >= outSize - 1) break;
      output[j++] = '\\';
      output[j++] = 'r';
    } else if (c == '\t') {
      if (j + 2 >= outSize - 1) break;
      output[j++] = '\\';
      output[j++] = 't';
    } else {
      output[j++] = c;
    }
  }
  output[j] = '\0';
}

void int32ToDMSString(int32_t coord_scaled, bool isLatitude, char *buffer, size_t bufSize) {
  // Séparer le signe
  bool isPositive = (coord_scaled >= 0);
  int32_t absCoord = isPositive ? coord_scaled : -coord_scaled;

  // Degrés
  int degrees = absCoord / 10000000;

  // Minutes
  int32_t minutes_part = absCoord % 10000000;
  int minutes = (minutes_part * 60) / 10000000;

  // Secondes (float pour précision)
  float seconds = ((minutes_part * 60.0f) / 10000000 - minutes) * 60.0f;

  // Direction
  char dir;
  if (isLatitude) dir = isPositive ? 'N' : 'S';
  else dir = isPositive ? 'E' : 'W';

  // Formater dans le buffer
  snprintf(buffer, bufSize, "%d°%d'%05.2f\"%c", degrees, minutes, seconds, dir);
}

static esp_err_t data_handler(httpd_req_t *req) {
  add_cors_headers(req);

  static char json[512];
  static char lat[20];
  static char lon[20];
  static char lat_json[32];
  static char lon_json[32];

  // Conversion DMS
  int32ToDMSString(latitude, true, lat, sizeof(lat));
  int32ToDMSString(longitude, false, lon, sizeof(lon));

  // Nettoyage fin de ligne
  lat[strcspn(lat, "\r\n")] = 0;
  lon[strcspn(lon, "\r\n")] = 0;

  // Échappement JSON
  escape_json(lat, lat_json, sizeof(lat_json));
  escape_json(lon, lon_json, sizeof(lon_json));

  float speedGPSFloat = speedGPS / 10.0f;

  float TempEspCPU = temperatureRead();

  float TempFloat = Temp / 10.0f;
  float HumFloat = Hum / 10.0f;
  float UbatFloat = Ubat / 100.0f;

  snprintf(json, sizeof(json),
           "{\"Sat\":%d,\"Lat\":\"%s\",\"Lon\":\"%s\",\"Alt\":%ld,\"speedGPS\":%.1f,\"Temp\":%.1f,\"Hum\":%.1f,\"Ubat\":%.2f,\"LumMoy\":%d,"
           "\"DistFront\":%d,\"DistBack\":%d,\"DistRight\":%d,\"DistLeft\":%d,\"ModMove\":%d,\"Speed\":%d,\"TempEspCPU\":%.1f}",
           Sat, lat_json, lon_json, altitude, speedGPSFloat, TempFloat, HumFloat, UbatFloat, LumMoy,
           DistFront, DistBack, DistRight, DistLeft, ModMove, speed, TempEspCPU);

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, strlen(json));
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 20;  // Augmentation du nombre de routes
  config.task_priority = 5;
  // config.stack_size = 4096;
  config.stack_size = 8192;  // default = 4096
  config.recv_wait_timeout = 10;
  config.send_wait_timeout = 10;
  config.max_open_sockets = 4;
  // config.max_open_sockets = 7;
  config.lru_purge_enable = true;

  httpd_uri_t index_uri = { "/", HTTP_GET, index_handler, NULL, false, false, NULL };
  httpd_uri_t go_uri = { "/go", HTTP_GET, go_handler, NULL, false, false, NULL };
  httpd_uri_t back_uri = { "/back", HTTP_GET, back_handler, NULL, false, false, NULL };
  httpd_uri_t stop_uri = { "/stop", HTTP_GET, stop_handler, NULL, false, false, NULL };
  httpd_uri_t left_uri = { "/left", HTTP_GET, left_handler, NULL, false, false, NULL };
  httpd_uri_t right_uri = { "/right", HTTP_GET, right_handler, NULL, false, false, NULL };
  httpd_uri_t ledon_uri = { "/ledon", HTTP_GET, ledon_handler, NULL, false, false, NULL };
  httpd_uri_t ledoff_uri = { "/ledoff", HTTP_GET, ledoff_handler, NULL, false, false, NULL };

  httpd_uri_t cam_left_uri = { "/cam_left", HTTP_GET, cam_left_handler, NULL, false, false, NULL };
  httpd_uri_t cam_right_uri = { "/cam_right", HTTP_GET, cam_right_handler, NULL, false, false, NULL };
  httpd_uri_t cam_center_uri = { "/cam_center", HTTP_GET, cam_center_handler, NULL, false, false, NULL };

  httpd_uri_t mod_0_uri = { "/mod_0", HTTP_GET, move_standart_handler, NULL, false, false, NULL };
  httpd_uri_t mod_1_uri = { "/mod_1", HTTP_GET, move_stop_obstacle_handler, NULL, false, false, NULL };

  httpd_uri_t up_speed_uri = { "/up_speed", HTTP_GET, up_speed_handler, NULL, false, false, NULL };
  httpd_uri_t down_speed_uri = { "/down_speed", HTTP_GET, down_speed_handler, NULL, false, false, NULL };

  httpd_uri_t status_uri = { "/status", HTTP_GET, status_handler, NULL, false, false, NULL };
  httpd_uri_t cmd_uri = { "/control", HTTP_GET, cmd_handler, NULL, false, false, NULL };
  httpd_uri_t capture_uri = { "/capture", HTTP_GET, capture_handler, NULL, false, false, NULL };
  httpd_uri_t uri_data = { "/data", HTTP_GET, data_handler, NULL, false, false, NULL };
  httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL, false, false, NULL };

  httpd_uri_t options_uri = { "/*", HTTP_OPTIONS, cors_options_handler, NULL, false, false, NULL };

  // Serial.printf("Starting web server on port: '%d'\n", config.server_port);

  if (httpd_start(&server_rover, &config) == ESP_OK) {
    httpd_register_uri_handler(server_rover, &index_uri);
    httpd_register_uri_handler(server_rover, &go_uri);
    httpd_register_uri_handler(server_rover, &back_uri);
    httpd_register_uri_handler(server_rover, &stop_uri);
    httpd_register_uri_handler(server_rover, &left_uri);
    httpd_register_uri_handler(server_rover, &right_uri);
    httpd_register_uri_handler(server_rover, &ledon_uri);
    httpd_register_uri_handler(server_rover, &ledoff_uri);

    httpd_register_uri_handler(server_rover, &cam_left_uri);
    httpd_register_uri_handler(server_rover, &cam_right_uri);
    httpd_register_uri_handler(server_rover, &cam_center_uri);

    httpd_register_uri_handler(server_rover, &mod_0_uri);
    httpd_register_uri_handler(server_rover, &mod_1_uri);

    httpd_register_uri_handler(server_rover, &up_speed_uri);
    httpd_register_uri_handler(server_rover, &down_speed_uri);

    httpd_register_uri_handler(server_rover, &status_uri);
    httpd_register_uri_handler(server_rover, &cmd_uri);
    httpd_register_uri_handler(server_rover, &capture_uri);
    httpd_register_uri_handler(server_rover, &uri_data);

    httpd_register_uri_handler(server_rover, &options_uri);
    // Serial.println("Web server started successfully.");
  } else {
    // Serial.println("Failed to start web server.");
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  config.task_priority = 6;  // Priorité plus haute
  config.stack_size = 8192;  // Stack plus large pour le stream

  // Serial.printf("Starting stream server on port: '%d'\n", config.server_port);

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    // Serial.println("Stream server started successfully.");
  } else {
    // Serial.println("Failed to start stream server.");
  }
}
