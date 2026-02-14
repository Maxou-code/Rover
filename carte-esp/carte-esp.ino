#include <esp_camera.h>
#include <esp_bt.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include "globals.hpp"

// Sélectionnez le modèle de caméra
#define CAMERA_MODEL_AI_THINKER
const char* ssid = "Rover";
const char* password = "12345678";

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

// Pin Lumière
int gpLed = 4;

// Variables
volatile int DistFront, DistBack, DistRight, DistLeft, LumMoy;
volatile int Temp, Hum, Ubat, Sat;
volatile int32_t latitude, longitude, altitude, speedGPS;

unsigned long lastFrameTime = 0;

void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  robot_setup();

  pinMode(gpLed, OUTPUT);
  digitalWrite(gpLed, LOW);

  // Désactive Bluetooth
  esp_bt_controller_disable();
  esp_bt_controller_deinit();

  camera_setup();
  wifi_setup();
  ota_setup();

  startCameraServer();
}

void loop() {
  ArduinoOTA.handle();

  static char buffer[512];
  static uint8_t index = 0;
  int count = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      buffer[index] = '\0';
      index = 0;
      parseFrame(buffer);
      count = 0;
    } else if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    }

    if (++count >= 50) {
      count = 0;
      yield();  // laisse respirer Wi-Fi/TCP/IP
    }
  }

  // FAILSAFE : plus de trame depuis > 1200 ms
  if (millis() - lastFrameTime > 1200) {
    robot_stop();
    // return;
  }

  // Sécurité obstacle
  if (ModMove == 1 && DistFront <= 20 && robot_fwd_val == true) {
    robot_stop();
    robot_fwd_val = false;
    // return;
  }

  yield();
}

inline char* nextField(char* p) {
  if (!p) return nullptr;
  char* c = strchr(p, ',');
  if (!c) return nullptr;
  *c = '\0';  // Terminer la chaîne à la virgule
  return c + 1;
}

void parseFrame(char* buf) {
  char* p = buf;

  DistFront = atoi(p); p = nextField(p);
  DistBack  = p ? atoi(p) : 0; p = nextField(p);
  DistRight = p ? atoi(p) : 0; p = nextField(p);
  DistLeft  = p ? atoi(p) : 0; p = nextField(p);

  LumMoy = p ? atoi(p) : 0; p = nextField(p);
  Temp   = p ? atoi(p) : 0; p = nextField(p);
  Hum    = p ? atoi(p) : 0; p = nextField(p);
  Ubat   = p ? atoi(p) : 0; p = nextField(p);

  Sat    = p ? atoi(p) : 0; p = nextField(p);
  latitude  = p ? atol(p) : 0; p = nextField(p);
  longitude = p ? atol(p) : 0; p = nextField(p);
  altitude  = p ? atol(p) : 0; p = nextField(p);
  speedGPS  = p ? atol(p) : 0;

  lastFrameTime = millis();
}

void camera_setup() {
  // Config Cam
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

  // Configuration optimale pour streaming fluide
  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;  // 640x480 si PSRAM
    config.jpeg_quality = 10;
    config.fb_count = 2;                    // Double buffering !
    config.grab_mode = CAMERA_GRAB_LATEST;  // IMPORTANT : toujours la frame la plus récente
  } else {
    config.frame_size = FRAMESIZE_CIF;  // 400x296 sans PSRAM
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }

  // Init caméra
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    // Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  // Taille réduite pour plus de FPS au démarrage
  // Après init, réglages fins du capteur
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    // Pas de changement de framesize après init
    s->set_brightness(s, 0);                  // -2 à 2
    s->set_contrast(s, 0);                    // -2 à 2
    s->set_saturation(s, 0);                  // -2 à 2
    s->set_special_effect(s, 0);              // 0 = pas d'effet
    s->set_whitebal(s, 1);                    // white balance auto
    s->set_awb_gain(s, 1);                    // auto gain
    s->set_wb_mode(s, 0);                     // auto
    s->set_exposure_ctrl(s, 1);               // auto exposition
    s->set_aec2(s, 0);                        // AEC DSP
    s->set_ae_level(s, 0);                    // -2 à 2
    s->set_aec_value(s, 300);                 // 0 à 1200
    s->set_gain_ctrl(s, 1);                   // auto gain
    s->set_agc_gain(s, 0);                    // 0 à 30
    s->set_gainceiling(s, (gainceiling_t)0);  // 0 à 6
    s->set_bpc(s, 0);                         // black pixel correction
    s->set_wpc(s, 1);                         // white pixel correction
    s->set_raw_gma(s, 1);                     // gamma correction
    s->set_lenc(s, 1);                        // lens correction
    s->set_hmirror(s, 1);                     // 1 = miroir horizontal
    s->set_vflip(s, 1);                       // 1 = flip vertical
    s->set_dcw(s, 1);                         // downsize enable
    s->set_colorbar(s, 0);                    // 0 = pas de colorbar
  }
}

void wifi_setup() {
  WiFi.mode(WIFI_AP);

  // WiFi.setSleep(false);

  WiFi.softAP(ssid, password, 1, false, 1);
  IPAddress ip = WiFi.softAPIP();

  // Serial.print("AP IP address: ");
  // Serial.println(ip);

  // Serial.print("Rover Ready! Use 'http://");
  // Serial.print(ip);
  // Serial.println("' to connect");
}

void ota_setup() {
  ArduinoOTA.setHostname("ESP32_Rover");
  ArduinoOTA.setPassword("123456");

  ArduinoOTA.onStart([]() {
    // const char* type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";

    // Serial.print("OTA Start: updating ");
    // Serial.println(type);

    Serial.flush();
    Serial.end();

    robot_stop();          // sécurité
    esp_camera_deinit();   // TRÈS recommandé
  });

  ArduinoOTA.onEnd([]() {
    // Serial.println("OTA End\n");
    delay(100);
    ESP.restart();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    // Serial.printf(
    //   "OTA Progress: %u%%\r",
    //   (progress * 100) / total);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    // Serial.printf("OTA Error[%u]: ", error);
    // switch (error) {
    //   case OTA_AUTH_ERROR: Serial.println("Auth Failed"); break;
    //   case OTA_BEGIN_ERROR: Serial.println("Begin Failed"); break;
    //   case OTA_CONNECT_ERROR: Serial.println("Connect Failed"); break;
    //   case OTA_RECEIVE_ERROR: Serial.println("Receive Failed"); break;
    //   case OTA_END_ERROR: Serial.println("End Failed"); break;
    // }
  });

  ArduinoOTA.setTimeout(120000);

  ArduinoOTA.begin();
  // Serial.println("OTA Ready");
}
