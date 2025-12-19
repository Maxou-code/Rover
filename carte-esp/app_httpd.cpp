#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"
#include <ESP32Servo.h>

#define LEFT_M0 13
#define LEFT_M1 12
#define RIGHT_M0 14
#define RIGHT_M1 15

#define Servo_CAM_PIN 2

Servo Servo_CAM;

int position_servo = 90;

int speed = 150;

int mod_move = 0;

extern int gpLed;

extern String WiFiAddr;

extern int D1, D2, D3, D4, LumMoy, Temp, Hum;
extern char latitude[20];
extern char longitude[20];

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
  ledcAttach(LEFT_M0, 2000, 8);  /* 2000 hz PWM, 8-bit resolution and range from 0 to 255 */
  ledcAttach(LEFT_M1, 2000, 8);  /* 2000 hz PWM, 8-bit resolution and range from 0 to 255 */
  ledcAttach(RIGHT_M0, 2000, 8); /* 2000 hz PWM, 8-bit resolution and range from 0 to 255 */
  ledcAttach(RIGHT_M1, 2000, 8); /* 2000 hz PWM, 8-bit resolution and range from 0 to 255 */

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
}

void robot_fwd() {
  if (mod_move == 1 && D1 <= 20) {
    // STOP
    robot_stop();
    return;
  }
  
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
  delay(20);
}

void camera_right() {
  if (position_servo > 0) {
    position_servo -= 10;
  }
  Servo_CAM.write(position_servo);
  delay(20);
}

void camera_center() {
  position_servo = 90;
  Servo_CAM.write(position_servo);
  delay(20);
}

void WheelAct(int nLf, int nLb, int nRf, int nRb);

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
httpd_handle_t camera_httpd = NULL;

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
    // Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)", (uint32_t)(_jpg_buf_len),
    //               (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
    //               avg_frame_time, 1000.0 / avg_frame_time);
  }

  last_frame = 0;
  return res;
}

static esp_err_t cmd_handler(httpd_req_t *req) {
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

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req) {
  static char json_response[1024];

  sensor_t *s = esp_camera_sensor_get();
  char *p = json_response;
  *p++ = '{';

  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
  p += sprintf(p, "\"quality\":%u,", s->status.quality);
  p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
  p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
  p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
  p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
  p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
  p += sprintf(p, "\"awb\":%u,", s->status.awb);
  p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
  p += sprintf(p, "\"aec\":%u,", s->status.aec);
  p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
  p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
  p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
  p += sprintf(p, "\"agc\":%u,", s->status.agc);
  p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
  p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
  p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
  p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
  p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
  p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
  p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
  p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
  p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

// Chaine = D1, D2, D3, D4, LumMoy, Temp, Hum;

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");

  String page = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">

<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=0">
    <title>Rover</title>
    <style>
    * {
        font-family: Arial;
        margin: 0;
        padding: 0;
    }

    body {
        display: flex;
        justify-content: space-evenly;
        align-items: center;
        height: 100vh;
        background: #2f2f2f;
        flex-direction: column;
    }

    h1 {
        color: white;
    }

    section {
        display: flex;
        width: 100%;
        justify-content: space-evenly;
    }

    div {
        background: rgba(0, 0, 0, 0.3);
        border-radius: 8px;
        display: flex;
        flex-direction: column;
        justify-content: center;
        gap: 15px;
        padding: 15px;
    }

    button {
        width: 140px;
        height: 40px;
        border-radius: 8px;
        border: none;
        font-weight: bold;
        cursor: pointer;
        transition: transform 0.1s ease-in-out;
    }

    button:active {
        transform: scale(0.9);
    }

    .container {
        color: white;
        width: 300px;
    }

    .container .mode {
        background-color: orange;
    }

    .video {
        transform: rotate(180deg);
        width: 720px;
        border-radius: 8px;
    }

    .image {
        text-align: center;
    }

    .image .orange {
        background-color: orange;
    }

    .controls {
        text-align: center;
        width: 300px;
    }

    .controls .blue {
        background-color: #0099FF;
        width: 90px;
        height: 80px;
    }

    .controls .red {
        background-color: #FB4040;
        width: 90px;
        height: 80px;
    }

    .controls .yellow {
        background-color: yellow;
    }

    /* Désactivation sélection texte */
    .no-select {
        user-select: none;
        -webkit-user-select: none;
        -ms-user-select: none;
    }
    </style>
    <script>
    function getsend(arg) {
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', arg + '?' + new Date().getTime(), true);
        xhttp.send();
    }

    function updateData() {
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
            if (this.readyState == 4 && this.status == 200) {
                let data = JSON.parse(this.responseText);
                lat.textContent = data.Lat;
                lon.textContent = data.Lon;
                temp.textContent = data.Temp;
                hum.textContent = data.Hum;
                lum.textContent = data.LumMoy;
                d1.textContent = data.d1;
                d2.textContent = data.d2;
                d3.textContent = data.d3;
                d4.textContent = data.d4;
                mode.textContent = data.mode;
            }
        };
        xhttp.open("GET", "/data", true);
        xhttp.send();
    }

    setInterval(updateData, 100);

    document.addEventListener("keydown", function(event) {
        switch (event.key) {
            // Déplacements 
            case "ArrowUp":
                getsend("go");
                break;
            case "ArrowDown":
                getsend("back");
                break;
            case "ArrowLeft":
                getsend("left");
                break;
            case "ArrowRight":
                getsend("right");
                break;

                // Espace = STOP immédiat
            case " ":
                getsend("stop");
                break;

                // Lumières            
            case "l":
            case "L":
                getsend("ledon");
                break;
            case "m":
            case "M":
                getsend("ledoff");
                break;

                // Caméra 
            case "f":
            case "F":
                getsend("cam_left");
                break;
            case "g":
            case "G":
                getsend("cam_center");
                break;
            case "h":
            case "H":
                getsend("cam_right");
                break;

                // Changement mode
            case "à":
            case "0":
                getsend("mod_0");
                break;
            case "&":
            case "1":
                getsend("mod_1");
                break;
        }
    });

    document.addEventListener("keyup", function(event) {
        switch (event.key) {
            case "ArrowUp":
            case "ArrowDown":
            case "ArrowLeft":
            case "ArrowRight":
                getsend("stop");
                break;
        }
    });
    </script>
</head>

<body>
    <h1>Rover</h1>
    <section>
        <div class="container no-select">
            <h3>Latitude : <span id="lat">NC</span></h3>
            <h3>Longitude : <span id="lon">NC</span></h3>
            <h3>Température : <span id="temp">0</span>°C</h3>
            <h3>Humidité : <span id="hum">0</span>%</h3>
            <h3>Luminosité : <span id="lum">0</span></h3>
            <h3>Distance avant : <span id="d1">0</span></h3>
            <h3>Distance arrière : <span id="d2">0</span></h3>
            <h3>Distance droite : <span id="d3">0</span></h3>
            <h3>Distance gauche : <span id="d4">0</span></h3>
            <br>
            <h3>Mode : <span id="mode">0</span></h3>
            <p>
                <button class="mode" onmousedown="getsend('mod_0')">0</button>
                <button class="mode" onmousedown="getsend('mod_1')">1</button>
            </p>
        </div>
        <div class="image no-select">
            <img src="http://)rawliteral" + WiFiAddr + R"rawliteral(:81/stream" class="video">
            <p>
                <button class="orange" onmousedown="getsend('cam_left')">Gauche</button>
                <button class="orange" onmousedown="getsend('cam_center')">Centre</button>
                <button class="orange" onmousedown="getsend('cam_right')">Droite</button>
            </p>
        </div>
        <div class="controls no-select">
            <p><button class="blue" onmousedown="getsend('go')" onmouseup="getsend('stop')">Avancer</button></p>
            <p>
                <button class="blue" onmousedown="getsend('left')" onmouseup="getsend('stop')">Gauche</button>
                <button class="red" onmousedown="getsend('stop')">Stop</button>
                <button class="blue" onmousedown="getsend('right')" onmouseup="getsend('stop')">Droite</button>
            </p>
            <p><button class="blue" onmousedown="getsend('back')" onmouseup="getsend('stop')">Reculer</button></p>
            <p>
                <button class="yellow" onmousedown="getsend('ledon')">Lumière ON</button>
                <button class="yellow" onmousedown="getsend('ledoff')">Lumière OFF</button>
            </p>
        </div>
    </section>
</body>

</html>
    )rawliteral";

  return httpd_resp_send(req, &page[0], strlen(&page[0]));
}

static esp_err_t go_handler(httpd_req_t *req) {
  //WheelAct(HIGH, LOW, HIGH, LOW);
  robot_fwd();
  Serial.println("Go");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t back_handler(httpd_req_t *req) {
  //WheelAct(LOW, HIGH, LOW, HIGH);
  robot_back();
  Serial.println("Back");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t left_handler(httpd_req_t *req) {
  //WheelAct(HIGH, LOW, LOW, HIGH);
  robot_left();
  Serial.println("Left");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}
static esp_err_t right_handler(httpd_req_t *req) {
  //WheelAct(LOW, HIGH, HIGH, LOW);
  robot_right();
  Serial.println("Right");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t stop_handler(httpd_req_t *req) {
  //WheelAct(LOW, LOW, LOW, LOW);
  robot_stop();
  Serial.println("Stop");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t ledon_handler(httpd_req_t *req) {
  digitalWrite(gpLed, HIGH);
  Serial.println("LED ON");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t ledoff_handler(httpd_req_t *req) {
  digitalWrite(gpLed, LOW);
  Serial.println("LED OFF");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t cam_left_handler(httpd_req_t *req) {
  camera_left();
  Serial.println("CAM LEFT");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t cam_right_handler(httpd_req_t *req) {
  camera_right();
  Serial.println("CAM RIGHT");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t cam_center_handler(httpd_req_t *req) {
  camera_center();
  Serial.println("CAM CENTER");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t move_standart_handler(httpd_req_t *req) {
  mod_move = 0;
  Serial.println("MOD MOVE 0");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t move_stop_obstacle_handler(httpd_req_t *req) {
  mod_move = 1;
  Serial.println("MOD MOVE 1");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t data_handler(httpd_req_t *req) {
  String lat = String(latitude);
  String lon = String(longitude);

  lat.replace("\"", "\\\"");
  lon.replace("\"", "\\\"");

  String json = "{\"Lat\":\"" + lat + "\",\"Lon\":\"" + lon + "\",\"Temp\":" + String(Temp) + ",\"Hum\":" + String(Hum) + ",\"LumMoy\":" + String(LumMoy) + ",\"d1\":" + String(D1) + ",\"d2\":" + String(D2) + ",\"d3\":" + String(D3) + ",\"d4\":" + String(D4) + ",\"mode\":" + String(mod_move) + "}";

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json.c_str(), json.length());
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 18;  // Augmentation du nombre de routes

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

  httpd_uri_t status_uri = { "/status", HTTP_GET, status_handler, NULL };
  httpd_uri_t cmd_uri = { "/control", HTTP_GET, cmd_handler, NULL };
  httpd_uri_t capture_uri = { "/capture", HTTP_GET, capture_handler, NULL };
  httpd_uri_t uri_data = { "/data", HTTP_GET, data_handler, NULL };
  httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };

  ra_filter_init(&ra_filter, 20);
  Serial.printf("Starting web server on port: '%d'\n", config.server_port);

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &go_uri);
    httpd_register_uri_handler(camera_httpd, &back_uri);
    httpd_register_uri_handler(camera_httpd, &stop_uri);
    httpd_register_uri_handler(camera_httpd, &left_uri);
    httpd_register_uri_handler(camera_httpd, &right_uri);
    httpd_register_uri_handler(camera_httpd, &ledon_uri);
    httpd_register_uri_handler(camera_httpd, &ledoff_uri);

    httpd_register_uri_handler(camera_httpd, &cam_left_uri);
    httpd_register_uri_handler(camera_httpd, &cam_right_uri);
    httpd_register_uri_handler(camera_httpd, &cam_center_uri);

    httpd_register_uri_handler(camera_httpd, &mod_0_uri);
    httpd_register_uri_handler(camera_httpd, &mod_1_uri);

    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &uri_data);
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
