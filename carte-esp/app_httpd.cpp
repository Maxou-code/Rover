#include <esp_http_server.h>
#include <esp_timer.h>
#include <esp_camera.h>
#include <img_converters.h>
#include <ESP32Servo.h>

#include "globals.hpp"

#define LEFT_M0 13
#define LEFT_M1 12
#define RIGHT_M0 14
#define RIGHT_M1 15

#define Servo_CAM_PIN 2

Servo Servo_CAM;

int position_servo = 90;

int speed = 100;

int ModMove = 0;
bool robot_fwd_val = false;

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
  const int pwmFreq = 20000; // 20 kHz
  const int pwmRes  = 8;     // 0–255

  ledcAttach(LEFT_M0, pwmFreq, pwmRes);
  ledcAttach(LEFT_M1, pwmFreq, pwmRes);
  ledcAttach(RIGHT_M0, pwmFreq, pwmRes);
  ledcAttach(RIGHT_M1, pwmFreq, pwmRes);

  Servo_CAM.setPeriodHertz(50);
  Servo_CAM.attach(Servo_CAM_PIN, 500, 2400);

  pinMode(33, OUTPUT);
  robot_stop();
  Servo_CAM.write(position_servo);
}

// Motor Control Functions
void robot_stop() {
  ledcWrite(LEFT_M0, 0);
  ledcWrite(LEFT_M1, 0);
  ledcWrite(RIGHT_M0, 0);
  ledcWrite(RIGHT_M1, 0);
  if (speed >= 100){
	speed = 100;
  }
  robot_fwd_val = false;
}

void robot_fwd() {
  if (ModMove == 1 && DistFront <= 20) {
	robot_stop();
	return;
  }

  robot_fwd_val = true;
  ledcWrite(LEFT_M0, 0);
  ledcWrite(LEFT_M1, speed);
  ledcWrite(RIGHT_M0, 0);
  ledcWrite(RIGHT_M1, speed);
}

void robot_back() {
  ledcWrite(LEFT_M0, speed);
  ledcWrite(LEFT_M1, 0);
  ledcWrite(RIGHT_M0, speed);
  ledcWrite(RIGHT_M1, 0);
}

void robot_right() {
  ledcWrite(LEFT_M0, 0);
  ledcWrite(LEFT_M1, speed);
  ledcWrite(RIGHT_M0, speed);
  ledcWrite(RIGHT_M1, 0);
}

void robot_left() {
  ledcWrite(LEFT_M0, speed);
  ledcWrite(LEFT_M1, 0);
  ledcWrite(RIGHT_M0, 0);
  ledcWrite(RIGHT_M1, speed);
}

void camera_left() {
  if (position_servo < 180) {
	position_servo += 10;
  }
  Servo_CAM.write(position_servo);
}

void camera_right() {
  if (position_servo > 0) {
	position_servo -= 10;
  }
  Servo_CAM.write(position_servo);
}

void camera_center() {
  position_servo = 90;
  Servo_CAM.write(position_servo);
}

typedef struct {
  size_t size;   //number of values used for filtering
  size_t index;  //current value index
  size_t count;  //value count
  int sum;
  int *values;  //array to be filled with values
} ra_filter_t;

typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t server_rover = NULL;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size) {
  memset(filter, 0, sizeof(ra_filter_t));

  filter->values = (int *)malloc(sample_size * sizeof(int));
  if (!filter->values) {
	return NULL;
  }
  memset(filter->values, 0, sample_size * sizeof(int));

  filter->size = sample_size;
  return filter;
}

static int ra_filter_run(ra_filter_t *filter, int value) {
  if (!filter->values) {
	return value;
  }
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index++;
  filter->index = filter->index % filter->size;
  if (filter->count < filter->size) {
	filter->count++;
  }
  return filter->sum / filter->count;
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
	j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
	return 0;
  }
  j->len += len;
  return len;
}

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  int64_t fr_start = esp_timer_get_time();

  fb = esp_camera_fb_get();
  if (!fb) {
	Serial.printf("Camera capture failed");
	httpd_resp_send_500(req);
	return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");

  size_t fb_len = 0;
  if (fb->format == PIXFORMAT_JPEG) {
	fb_len = fb->len;
	res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  } else {
	jpg_chunking_t jchunk = { req, 0 };
	res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
	httpd_resp_send_chunk(req, NULL, 0);
	fb_len = jchunk.len;
  }
  esp_camera_fb_return(fb);
  int64_t fr_end = esp_timer_get_time();
  // Serial.printf("JPG: %uB %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  static int64_t last_frame = 0;
  if (!last_frame) {
	last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
	return res;
  }

  while (true) {
	fb = esp_camera_fb_get();
	if (!fb) {
	  Serial.printf("Camera capture failed");
	  res = ESP_FAIL;
	} else {
	  if (fb->format != PIXFORMAT_JPEG) {
		bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
		esp_camera_fb_return(fb);
		fb = NULL;
		if (!jpeg_converted) {
		  Serial.printf("JPEG compression failed");
		  res = ESP_FAIL;
		}
	  } else {
		_jpg_buf_len = fb->len;
		_jpg_buf = fb->buf;
	  }
	}
	if (res == ESP_OK) {
	  size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
	  res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
	}
	if (res == ESP_OK) {
	  res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
	}
	if (res == ESP_OK) {
	  res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
	}
	if (fb) {
	  esp_camera_fb_return(fb);
	  fb = NULL;
	  _jpg_buf = NULL;
	} else if (_jpg_buf) {
	  free(_jpg_buf);
	  _jpg_buf = NULL;
	}
	if (res != ESP_OK) {
	  break;
	}
	int64_t fr_end = esp_timer_get_time();

	int64_t frame_time = fr_end - last_frame;
	last_frame = fr_end;
	frame_time /= 1000;
	uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
  }

  last_frame = 0;
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
		s->status.raw_gma, s->status.lenc, s->status.hmirror, s->status.dcw, s->status.colorbar
	);

	if (len < 0) {
		return httpd_resp_send_500(req);
	}

	httpd_resp_set_type(req, "application/json");
	return httpd_resp_send(req, json_response, len);
}

// Chaine = DistFront, DistBack, DistRight, DistLeft, LumMoy, Temp, Hum;

static esp_err_t cors_options_handler(httpd_req_t *req) {
	add_cors_headers(req);
	return httpd_resp_send(req, NULL, 0);  // réponse vide
}

static esp_err_t index_handler(httpd_req_t *req) {
  add_cors_headers(req);
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t go_handler(httpd_req_t *req) {
  add_cors_headers(req);
  robot_fwd();
  Serial.println("Go");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t back_handler(httpd_req_t *req) {
  add_cors_headers(req);
  robot_back();
  Serial.println("Back");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t left_handler(httpd_req_t *req) {
  add_cors_headers(req);
  robot_left();
  Serial.println("Left");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}
static esp_err_t right_handler(httpd_req_t *req) {
  add_cors_headers(req);
  robot_right();
  Serial.println("Right");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t stop_handler(httpd_req_t *req) {
  add_cors_headers(req);
  robot_stop();
  Serial.println("Stop");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t ledon_handler(httpd_req_t *req) {
  add_cors_headers(req);
  digitalWrite(gpLed, HIGH);
  Serial.println("LED ON");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t ledoff_handler(httpd_req_t *req) {
  add_cors_headers(req);
  digitalWrite(gpLed, LOW);
  Serial.println("LED OFF");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t cam_left_handler(httpd_req_t *req) {
  add_cors_headers(req);
  camera_left();
  Serial.println("CAM LEFT");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t cam_right_handler(httpd_req_t *req) {
  add_cors_headers(req);
  camera_right();
  Serial.println("CAM RIGHT");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t cam_center_handler(httpd_req_t *req) {
  add_cors_headers(req);
  camera_center();
  Serial.println("CAM CENTER");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t move_standart_handler(httpd_req_t *req) {
  add_cors_headers(req);
  ModMove = 0;
  Serial.println("MOD MOVE 0");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t move_stop_obstacle_handler(httpd_req_t *req) {
  add_cors_headers(req);
  ModMove = 1;
  Serial.println("MOD MOVE 1");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t up_speed_handler(httpd_req_t *req) {
  add_cors_headers(req);
  if(speed < 255){speed += 5;}
  Serial.println("Speep up");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t down_speed_handler(httpd_req_t *req) {
  add_cors_headers(req);
  if(speed > 0){speed -= 5;}
  Serial.println("Speed down");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

void escape_json(const char *in, char *out, size_t out_size) {
  size_t j = 0;

  for (size_t i = 0; in[i] && j + 2 < out_size; i++) {
	if (in[i] == '"' || in[i] == '\\') {
	  out[j++] = '\\';
	}
	out[j++] = in[i];
  }

  out[j] = '\0';
}

static esp_err_t data_handler(httpd_req_t *req) {
  add_cors_headers(req);
  static char json[256];
  static char lat[32];
  static char lon[32];

  escape_json(latitude, lat, sizeof(lat));
  escape_json(longitude, lon, sizeof(lon));

  snprintf(json, sizeof(json),
	"{\"Lat\":\"%s\",\"Lon\":\"%s\",\"Temp\":%d,\"Hum\":%d,\"LumMoy\":%d,"
	"\"DistFront\":%d,\"DistBack\":%d,\"DistRight\":%d,\"DistLeft\":%d,\"ModMove\":%d,\"Speed\":%d}",
	lat, lon, Temp, Hum, LumMoy,
	DistFront, DistBack, DistRight, DistLeft, ModMove, speed
  );

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, strlen(json));
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 19;  // Augmentation du nombre de routes

  httpd_uri_t index_uri = { "/", HTTP_GET, index_handler, NULL };
  httpd_uri_t go_uri = { "/go", HTTP_GET, go_handler, NULL };
  httpd_uri_t back_uri = { "/back", HTTP_GET, back_handler, NULL };
  httpd_uri_t stop_uri = { "/stop", HTTP_GET, stop_handler, NULL };
  httpd_uri_t left_uri = { "/left", HTTP_GET, left_handler, NULL };
  httpd_uri_t right_uri = { "/right", HTTP_GET, right_handler, NULL };
  httpd_uri_t ledon_uri = { "/ledon", HTTP_GET, ledon_handler, NULL };
  httpd_uri_t ledoff_uri = { "/ledoff", HTTP_GET, ledoff_handler, NULL };

  httpd_uri_t cam_left_uri = { "/cam_left", HTTP_GET, cam_left_handler, NULL };
  httpd_uri_t cam_right_uri = { "/cam_right", HTTP_GET, cam_right_handler, NULL };
  httpd_uri_t cam_center_uri = { "/cam_center", HTTP_GET, cam_center_handler, NULL };

  httpd_uri_t mod_0_uri = { "/mod_0", HTTP_GET, move_standart_handler, NULL };
  httpd_uri_t mod_1_uri = { "/mod_1", HTTP_GET, move_stop_obstacle_handler, NULL };

  httpd_uri_t up_speed_uri = { "/up_speed", HTTP_GET, up_speed_handler, NULL };
  httpd_uri_t down_speed_uri = { "/down_speed", HTTP_GET, down_speed_handler, NULL };

  httpd_uri_t status_uri = { "/status", HTTP_GET, status_handler, NULL };
  httpd_uri_t cmd_uri = { "/control", HTTP_GET, cmd_handler, NULL };
  httpd_uri_t capture_uri = { "/capture", HTTP_GET, capture_handler, NULL };
  httpd_uri_t uri_data = { "/data", HTTP_GET, data_handler, NULL };
  httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };

  httpd_uri_t options_uri = { "/*", HTTP_OPTIONS, cors_options_handler, NULL };

  ra_filter_init(&ra_filter, 20);
  Serial.printf("Starting web server on port: '%d'\n", config.server_port);

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
	Serial.println("Web server started successfully.");
  } else {
	Serial.println("Failed to start web server.");
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  Serial.printf("Starting stream server on port: '%d'\n", config.server_port);

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
	httpd_register_uri_handler(stream_httpd, &stream_uri);
	Serial.println("Stream server started successfully.");
  } else {
	Serial.println("Failed to start stream server.");
  }
}
