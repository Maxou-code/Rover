#include <esp_camera.h>
#include <esp_bt.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include "globals.hpp"

// Sélectionnez le modèle de caméra
#define CAMERA_MODEL_AI_THINKER
const char *ssid = "Rover";
const char *password = "12345678";

extern void robot_stop();
extern void robot_setup();

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIODistBackPIO_NUM 26
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
int DistFront, DistBack, DistRight, DistLeft, LumMoy, Temp, Hum;
char latitude[20];
char longitude[20];

unsigned long lastFrameTime = 0;

void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  robot_setup();
  pinMode(gpLed, OUTPUT);
  digitalWrite(gpLed, LOW);

  // Désactive Bluetooth
  esp_bt_controller_disable();
  esp_bt_controller_deinit();

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
  config.pin_sscb_sda = SIODistBackPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Initialisation avec des spécifications élevées pour pré-allouer des buffers plus grands
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Initialisation de la caméra
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Réduire la taille d'image pour obtenir une fréquence d'images initiale plus élevée
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);

  // Initialisation Wi-Fi
  WiFi.softAP(ssid, password);

  IPAddress ip = WiFi.softAPIP();

  Serial.print("AP IP address: ");
  Serial.println(ip);

  Serial.print("Rover Ready! Use 'http://");
  Serial.print(ip);
  Serial.println("' to connect");

  // Configure OTA
  ArduinoOTA.setHostname("ESP32_Rover");
  ArduinoOTA.setPassword("123456");

  ArduinoOTA.onStart([]() {
    const char* type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";

    Serial.print("OTA Start: updating ");
    Serial.println(type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA End\n");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    switch (error) {
      case OTA_AUTH_ERROR: Serial.println("Auth Failed"); break;
      case OTA_BEGIN_ERROR: Serial.println("Begin Failed"); break;
      case OTA_CONNECT_ERROR: Serial.println("Connect Failed"); break;
      case OTA_RECEIVE_ERROR: Serial.println("Receive Failed"); break;
      case OTA_END_ERROR: Serial.println("End Failed"); break;
    }
  });

  ArduinoOTA.begin();
  Serial.println("OTA Ready");

  startCameraServer();
}

void loop() {
  ArduinoOTA.handle();

  static char buffer[128];
  static uint8_t index = 0;

  // Lecture série non bloquante caractère par caractère
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      buffer[index] = '\0';  // fin de chaîne
      index = 0;

      int parsed = sscanf(
        buffer,
        "%d,%d,%d,%d,%d,%d,%d,%19[^,],%19s",
        &DistFront, &DistBack, &DistRight, &DistLeft,
        &LumMoy, &Temp, &Hum,
        latitude, longitude
      );

      if (parsed == 9) {
        lastFrameTime = millis();  // trame valide
      } else {
        Serial.println("Trame invalide !");
      }

    } else if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    }
    // sinon : caractère ignoré (overflow évité)
  }

  // FAILSAFE : plus de trame depuis > 1200 ms
  if (millis() - lastFrameTime > 1200) {
    robot_stop();
    return;
  }

  // Sécurité obstacle
  if (ModMove == 1 && DistFront <= 20 && robot_fwd_val == true) {
    robot_stop();
    robot_fwd_val = false;
    return;
  }
}
