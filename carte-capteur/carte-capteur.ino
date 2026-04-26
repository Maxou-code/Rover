#include <DHT.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include <Arduino.h>
#include <math.h>

// #define PIN_LED 4

#define PIN_PDT A8

#define EN_GPS_PIN 35
#define PP_GPS_PIN 37

#define AIN1_PIN 25
#define AIN2_PIN 23

#define STBY_PIN 27

#define BIN1_PIN 29
#define BIN2_PIN 31

#define PWM_SERVO_CAM_X 6
#define PWM_SERVO_CAM_Y 7

#define DHT_PIN 2
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);
float temp, hum;

TinyGPSPlus gps;

// --------------------------
// UBX pour activer Galileo
// Compatible u-blox M8N / M9N
// --------------------------
uint8_t enableGalileo[] = {
  0xB5,0x62,0x06,0x3E,0x3C,0x00,
  0x00,0x00,0x20,0x07,
  0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,
  0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,
  0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,
  0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,
  0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,
  0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,
  0x00,0x00
};

// UBX pour sauvegarder en mémoire permanente
uint8_t saveConfig[] = {
  0xB5,0x62,0x06,0x09,0x0D,0x00,
  0x00,0x00,0x00,0x00,
  0xFF,0xFF,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00
};

Servo ServoCamX;
Servo ServoCamY;

float Vref = 5.0;
float diviseur = 3.07;
float Ubat;

int Temp10;   // température ×10
int Hum10;    // humidité ×10
int Ubat100;  // tension ×100

int photo_1 = A0;
int photo_2 = A1;
int photo_3 = A2;
int photo_4 = A3;

int val_photo_1, val_photo_2, val_photo_3, val_photo_4, val_photo_moyen;

int Echo_Capteur_US_AD = 24;
int Trig_Capteur_US_AD = 22;

int Echo_Capteur_US_Ar = 30;
int Trig_Capteur_US_Ar = 28;

int Echo_Capteur_US_D = 36;
int Trig_Capteur_US_D = 34;

int Echo_Capteur_US_G = 42;
int Trig_Capteur_US_G = 40;

int Echo_Capteur_US_AG = 48;
int Trig_Capteur_US_AG = 46;

int Distance_AD, Distance_Ar, Distance_D, Distance_G, Distance_AG;

int Distance_A;

int sat;
double lat = 0;
double lng = 0;

int32_t latE7 = 0;
int32_t lngE7 = 0;
int32_t alt_int = 0;
int32_t spd_int = 0;

int AIN1_val = 0;
int AIN2_val = 0;

int BIN1_val = 0;
int BIN2_val = 0;

int SERVO_X_val = 90;
int SERVO_Y_val = 90;

int NEW_SERVO_X_val;
int NEW_SERVO_Y_val;

unsigned long lastSensorUpdate = 0;
const unsigned long SENSOR_INTERVAL = 200;  // ms

unsigned long lastUSGroup1 = 0;
unsigned long lastUSGroup2 = 0;
unsigned long lastDHT = 0;
unsigned long lastLight = 0;

const unsigned long INTERVAL_US = 100;
const unsigned long INTERVAL_DHT = 250;
const unsigned long INTERVAL_LIGHT = 100;

char serial2Buffer[64];
uint8_t serial2Index = 0;

unsigned long lastSensorTime = 0; // Temps de la dernière lecture
const unsigned long sensorInterval = 50; // Intervalle en ms
int sensorIndex = 0;

const int PHOTO_SAMPLES = 10; // Nombre de lectures pour la moyenne glissante

int photo1Buffer[PHOTO_SAMPLES] = {0};
int photo2Buffer[PHOTO_SAMPLES] = {0};
int photo3Buffer[PHOTO_SAMPLES] = {0};
int photo4Buffer[PHOTO_SAMPLES] = {0};
int photoIndex = 0;

// Fonction pour convertir la valeur analogique en lux
int analogToLux(int val) {
    if (val <= 0) return 0; // éviter division par zéro

    const float R_FIXED = 10000.0;  // Résistance fixe du diviseur (10kΩ)
    const float K = 500000.0;       // Constante typique de la LDR (ohms)
    const float ALPHA = 0.7;        // Exposant typique

    // Calcul de la tension lue
    float Vout = val * 5.0 / 1023.0;

    // Calcul de la résistance de la LDR
    float R_LDR = R_FIXED * (5.0 - Vout) / Vout;

    // Conversion résistance → lux
    float lux = pow(K / R_LDR, 1.0 / ALPHA);

    return (int)lux;  // renvoie un entier
}

unsigned long lastUS = 0;

int usIndex = 0;

// valeurs filtrées
float fDistance_AD = 0;
float fDistance_AG = 0;
float fDistance_D = 0;
float fDistance_G = 0;
float fDistance_Ar = 0;

int Distance_fast(int trig, int echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    long duration = pulseIn(echo, HIGH, 15000); // timeout réduit

    if (duration == 0) return -1;
    return duration / 58;
}

float filterEMA(float oldVal, float newVal, float alpha = 0.5) {
    if (newVal == -1) return oldVal;
    return alpha * newVal + (1 - alpha) * oldVal;
}

void setup() {
  Serial1.begin(9600);    // GPS
  Serial2.begin(115200);  // ESP

  pinMode(EN_GPS_PIN, OUTPUT);
  digitalWrite(EN_GPS_PIN, HIGH);

  delay(2000);

  Serial1.write(enableGalileo, sizeof(enableGalileo));
  delay(500);

  // Sauvegarder en mémoire
  Serial1.write(saveConfig, sizeof(saveConfig));

  dht.begin();

  pinMode(photo_1, INPUT);
  pinMode(photo_2, INPUT);
  pinMode(photo_3, INPUT);
  pinMode(photo_4, INPUT);

  pinMode(Echo_Capteur_US_AD, INPUT);
  pinMode(Trig_Capteur_US_AD, OUTPUT);
  pinMode(Echo_Capteur_US_Ar, INPUT);
  pinMode(Trig_Capteur_US_Ar, OUTPUT);
  pinMode(Echo_Capteur_US_D, INPUT);
  pinMode(Trig_Capteur_US_D, OUTPUT);
  pinMode(Echo_Capteur_US_G, INPUT);
  pinMode(Trig_Capteur_US_G, OUTPUT);
  pinMode(Echo_Capteur_US_AG, INPUT);
  pinMode(Trig_Capteur_US_AG, OUTPUT);

  pinMode(STBY_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);

  digitalWrite(STBY_PIN, HIGH);
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN2_PIN, LOW);

  ServoCamX.attach(PWM_SERVO_CAM_X);
  ServoCamX.write(SERVO_X_val);

  ServoCamY.attach(PWM_SERVO_CAM_Y);
  ServoCamY.write(SERVO_Y_val);

  // pinMode(PIN_LED, OUTPUT);
  // digitalWrite(PIN_LED, HIGH);
}

void loop() {

  // 🔥 PRIORITÉ MAX AUX COMMANDES
  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '\n') {
      serial2Buffer[serial2Index] = '\0';

      int parsed = sscanf(serial2Buffer, "%d,%d,%d,%d,%d,%d",
        &AIN1_val, &AIN2_val, &BIN1_val, &BIN2_val,
        &NEW_SERVO_X_val, &NEW_SERVO_Y_val
      );

      if (parsed == 6) updateMotors();

      serial2Index = 0;
    } else {
      if (serial2Index < sizeof(serial2Buffer) - 1) {
        serial2Buffer[serial2Index++] = c;
      }
    }
  }

  // Capteurs après
  updateSensors();

  // GPS
  while (Serial1.available()) gps.encode(Serial1.read());
}

void updateMotors() {
  digitalWrite(AIN1_PIN, AIN1_val);
  digitalWrite(AIN2_PIN, AIN2_val);
  digitalWrite(BIN1_PIN, BIN1_val);
  digitalWrite(BIN2_PIN, BIN2_val);

  if (abs(NEW_SERVO_X_val - SERVO_X_val) > 3) {
    ServoCamX.write(NEW_SERVO_X_val);
    SERVO_X_val = NEW_SERVO_X_val;
  }

  if (abs(NEW_SERVO_Y_val - SERVO_Y_val) > 3) {
    ServoCamY.write(NEW_SERVO_Y_val);
    SERVO_Y_val = NEW_SERVO_Y_val;
  }
}

void updateSensors() {
  unsigned long now = millis();

  // =========================
  // ULTRASON (1 capteur à la fois)
  // =========================
  if (now - lastUS >= INTERVAL_US) {
    lastUS = now;

    int d;

    switch (usIndex) {
      case 0:
        d = Distance_fast(Trig_Capteur_US_G, Echo_Capteur_US_G);
        fDistance_G = filterEMA(fDistance_G, d);
        break;

      case 1:
        d = Distance_fast(Trig_Capteur_US_D, Echo_Capteur_US_D);
        fDistance_D = filterEMA(fDistance_D, d);
        break;

      case 2:
        d = Distance_fast(Trig_Capteur_US_AD, Echo_Capteur_US_AD);
        fDistance_AD = filterEMA(fDistance_AD, d);
        break;

      case 3:
        d = Distance_fast(Trig_Capteur_US_AG, Echo_Capteur_US_AG);
        fDistance_AG = filterEMA(fDistance_AG, d);
        break;

      case 4:
        d = Distance_fast(Trig_Capteur_US_Ar, Echo_Capteur_US_Ar);
        fDistance_Ar = filterEMA(fDistance_Ar, d);
        break;
    }

    usIndex++;
    if (usIndex > 4) usIndex = 0;
  }

  // Conversion en int
  Distance_G = (int)fDistance_G;
  Distance_D = (int)fDistance_D;
  Distance_AD = (int)fDistance_AD;
  Distance_AG = (int)fDistance_AG;
  Distance_Ar = (int)fDistance_Ar;

  // Fusion avant
  Distance_A =
      (Distance_AD == -1) ? Distance_AG :
      (Distance_AG == -1) ? Distance_AD :
      min(Distance_AD, Distance_AG);

  // =========================
  // LUMIÈRE + BATTERIE
  // =========================
  if (now - lastLight >= INTERVAL_LIGHT) {
    lastLight = now;

    photo1Buffer[photoIndex] = analogRead(photo_1);
    photo2Buffer[photoIndex] = analogRead(photo_2);
    photo3Buffer[photoIndex] = analogRead(photo_3);
    photo4Buffer[photoIndex] = analogRead(photo_4);

    photoIndex = (photoIndex + 1) % PHOTO_SAMPLES;

    long sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0;
    for (int i = 0; i < PHOTO_SAMPLES; i++) {
      sum1 += photo1Buffer[i];
      sum2 += photo2Buffer[i];
      sum3 += photo3Buffer[i];
      sum4 += photo4Buffer[i];
    }

    val_photo_1 = sum1 / PHOTO_SAMPLES;
    val_photo_2 = sum2 / PHOTO_SAMPLES;
    val_photo_3 = sum3 / PHOTO_SAMPLES;
    val_photo_4 = sum4 / PHOTO_SAMPLES;

    val_photo_moyen = (val_photo_1 + val_photo_2 + val_photo_3 + val_photo_4) / 4;
    val_photo_moyen = analogToLux(val_photo_moyen);

    // Batterie (rapide)
    long somme = 0;
    for (int i = 0; i < 5; i++) {
      somme += analogRead(PIN_PDT);
    }

    float val = somme / 5.0;
    Ubat = val * (Vref / 1023.0) * diviseur;
    Ubat100 = (int)(Ubat * 100.0f + 0.5f);
  }

  // =========================
  // DHT
  // =========================
  if (now - lastDHT >= INTERVAL_DHT) {
    lastDHT = now;

    temp = dht.readTemperature();
    hum = dht.readHumidity();

    if (!isnan(temp)) Temp10 = (int)(temp * 10.0f);
    if (!isnan(hum)) Hum10 = (int)(hum * 10.0f);
  }

  // =========================
  // GPS
  // =========================
  sat = gps.satellites.value();

  if (gps.location.isValid() && gps.location.age() < 2000 && sat >= 6 && gps.hdop.hdop() < 2.5) {
    latE7 = (int32_t)(gps.location.lat() * 10000000.0);
    lngE7 = (int32_t)(gps.location.lng() * 10000000.0);
  }

  if (gps.altitude.isValid() && gps.altitude.age() < 2000 && sat >= 6) {
    alt_int = (int32_t)gps.altitude.meters();
  }

  if (gps.speed.isValid() && gps.speed.age() < 2000 && sat >= 6) {
    spd_int = (int32_t)(gps.speed.kmph() * 10.0);
  }

  // =========================
  // ENVOI RAPIDE
  // =========================
  char buffer[128];
  sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%ld,%ld\n",
          Distance_A, Distance_Ar, Distance_D, Distance_G,
          val_photo_moyen, Temp10, Hum10, Ubat100,
          sat, latE7, lngE7, alt_int, spd_int);

  Serial2.print(buffer);
}