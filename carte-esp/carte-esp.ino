#include "esp_camera.h"
#include <WiFi.h>

// Select camera model
#define CAMERA_MODEL_AI_THINKER
const char *ssid1 = "Rover";
const char *password1 = "12345678";

extern void robot_stop();
extern void robot_setup();

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#else
#error "Camera model not selected"
#endif
extern int gpLed = 4;  // Light
extern String WiFiAddr = "";

extern int mod_move;
extern bool robot_fwd_val;

String data;

// Variables
int D1, D2, D3, D4, LumMoy, Temp, Hum;
char latitude[20];
char longitude[20];

void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  robot_setup();
  pinMode(gpLed, OUTPUT);  //Light
  digitalWrite(gpLed, LOW);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  //drop down frame size for higher initial frame rate
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);

  WiFi.softAP(ssid1, password1);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.softAPIP());
  WiFiAddr = WiFi.softAPIP().toString();
  Serial.println("' to connect");
  startCameraServer();
  digitalWrite(33, LOW);
}

unsigned long lastFrameTime = 0;

void loop() {
  // Lecture série non bloquante
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim(); // enlève \r et espaces

    int parsed = sscanf(
      data.c_str(),
      "%d,%d,%d,%d,%d,%d,%d,%[^,],%s",
      &D1, &D2, &D3, &D4,
      &LumMoy, &Temp, &Hum,
      latitude, longitude
    );

    if (parsed == 9) {
      lastFrameTime = millis();  // trame valide reçue
    } else {
      Serial.println("Trame invalide !");
    }
  }

  // FAILSAFE : plus de trame depuis > 1200 ms
  if (millis() - lastFrameTime > 1200) {
    robot_stop();
    return;
  }

  if (mod_move == 1 && D1 <= 20 && robot_fwd_val == true) {
    // STOP
    robot_stop();
    robot_fwd_val = false;
    return;
  }
}
