#include <DHT.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include <Arduino.h>
#include <math.h>

// =====================================================================
// PINS
// =====================================================================
#define PIN_PDT       A8

#define EN_GPS_PIN    35
#define PP_GPS_PIN    37

#define AIN1_PIN      25
#define AIN2_PIN      23
#define STBY_PIN      27
#define BIN1_PIN      29
#define BIN2_PIN      31

#define PWM_SERVO_CAM_X  6
#define PWM_SERVO_CAM_Y  7

#define DHT_PIN       2
#define DHT_TYPE      DHT22

// =====================================================================
// CONSTANTES ULTRASON
// =====================================================================
// Timeout pulseIn en µs → distance max correspondante
// À 343 m/s : d_max = (timeout * 343e-6) / 2
// 25 000 µs → ~4,3 m  (bien au-delà du HC-SR04 qui plafonne à ~4 m)
#define US_TIMEOUT_US    25000UL

// Distance renvoyée quand on atteint le timeout (objet trop loin ou absent)
#define US_MAX_DIST_CM   400

// Intervalle entre deux déclenchements (1 capteur à la fois, round-robin)
// ≥ 60 ms recommandé par le datasheet HC-SR04
#define INTERVAL_US      60UL

#define INTERVAL_DHT     250UL
#define INTERVAL_LIGHT   100UL

// =====================================================================
// EMA α  (0 < α ≤ 1 — plus grand = réaction plus rapide)
// =====================================================================
#define EMA_ALPHA  0.5f

// =====================================================================
// OBJETS GLOBAUX
// =====================================================================
DHT dht(DHT_PIN, DHT_TYPE);
TinyGPSPlus gps;
Servo ServoCamX;
Servo ServoCamY;

// =====================================================================
// UBX — Galileo + sauvegarde
// =====================================================================
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
uint8_t saveConfig[] = {
  0xB5,0x62,0x06,0x09,0x0D,0x00,
  0x00,0x00,0x00,0x00,
  0xFF,0xFF,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00
};

// =====================================================================
// CAPTEURS — VARIABLES
// =====================================================================

// DHT — on garde temp/hum valides même si une lecture échoue
float temp = 20.0f;   // valeur par défaut raisonnable pour la vitesse du son
float hum  = 50.0f;
int Temp10 = 200;
int Hum10  = 500;

// Batterie
const float Vref     = 5.0f;
const float diviseur = 3.07f;
float Ubat   = 0.0f;
int   Ubat100 = 0;

// Photorésistances
const int photo_1 = A0;
const int photo_2 = A1;
const int photo_3 = A2;
const int photo_4 = A3;

const int PHOTO_SAMPLES = 10;
int photo1Buffer[PHOTO_SAMPLES] = {0};
int photo2Buffer[PHOTO_SAMPLES] = {0};
int photo3Buffer[PHOTO_SAMPLES] = {0};
int photo4Buffer[PHOTO_SAMPLES] = {0};
int photoIndex = 0;

int val_photo_1 = 0, val_photo_2 = 0, val_photo_3 = 0, val_photo_4 = 0;
int val_photo_moyen = 0;

// Ultrason — broches
struct USSensor {
  int trig;
  int echo;
};

const USSensor usSensors[] = {
  {22, 24},  // 0 : AD (avant-droit)
  {28, 30},  // 1 : Ar (arrière)
  {34, 36},  // 2 : D  (droit)
  {40, 42},  // 3 : G  (gauche)
  {46, 48}   // 4 : AG (avant-gauche)
};
const int US_COUNT = 5;

float fDist[US_COUNT] = {(float)US_MAX_DIST_CM};  // initialisé à la distance max
int   iDist[US_COUNT] = {US_MAX_DIST_CM};

// Distances nommées (indices)
#define IDX_AD 0
#define IDX_Ar 1
#define IDX_D  2
#define IDX_G  3
#define IDX_AG 4

int Distance_A  = US_MAX_DIST_CM;  // fusion avant

// GPS
int     sat    = 0;
int32_t latE7  = 0;
int32_t lngE7  = 0;
int32_t alt_int = 0;
int32_t spd_int = 0;

// Moteurs / servos
int AIN1_val = 0, AIN2_val = 0;
int BIN1_val = 0, BIN2_val = 0;
int SERVO_X_val = 90, SERVO_Y_val = 90;

// Timers
unsigned long lastUS   = 0;
unsigned long lastDHT  = 0;
unsigned long lastLight = 0;
int usIndex = 0;

// Buffer Serial2
char serial2Buffer[64];
uint8_t serial2Index = 0;

// =====================================================================
// FONCTIONS UTILITAIRES
// =====================================================================

// Conversion analogique → lux (LDR sur diviseur de tension 10 kΩ, 5 V)
int analogToLux(int val) {
  if (val <= 0) return 0;
  const float R_FIXED = 10000.0f;
  const float K       = 500000.0f;
  const float ALPHA   = 0.7f;
  float Vout = val * 5.0f / 1023.0f;
  if (Vout <= 0.0f) return 0;
  float R_LDR = R_FIXED * (5.0f - Vout) / Vout;
  return (int)powf(K / R_LDR, 1.0f / ALPHA);
}

// Filtre EMA — renvoie oldVal inchangé si newVal == -1
float filterEMA(float oldVal, float newVal, float alpha = EMA_ALPHA) {
  if (newVal < 0) return oldVal;
  return alpha * newVal + (1.0f - alpha) * oldVal;
}

// Mesure ultrason avec correction thermique de la vitesse du son.
// Retourne US_MAX_DIST_CM si le timeout est atteint (objet absent / trop loin).
int Distance_fast(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, US_TIMEOUT_US);

  // Timeout → distance max
  if (duration == 0) return US_MAX_DIST_CM;

  // Vitesse du son corrigée par la température (m/s)
  float speed = 331.0f + (0.6f * temp);

  // distance = (durée en s × vitesse) / 2, convertie en cm
  return (int)((duration * 0.000001f * speed * 0.5f) * 100.0f);
}

// =====================================================================
// MOTEURS
// =====================================================================
void updateMotors() {
  digitalWrite(AIN1_PIN, AIN1_val);
  digitalWrite(AIN2_PIN, AIN2_val);
  digitalWrite(BIN1_PIN, BIN1_val);
  digitalWrite(BIN2_PIN, BIN2_val);
  
  // les valeurs sont déjà parsées dans serial2Buffer
  // (appelé depuis le parsing, on utilise les globales NEW_SERVO_*)
}

// =====================================================================
// CAPTEURS
// =====================================================================
void updateSensors() {
  unsigned long now = millis();

  // --- DHT ---
  if (now - lastDHT >= INTERVAL_DHT) {
    lastDHT = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    // On ne met à jour que si la lecture est valide → temp/hum restent cohérents
    if (!isnan(t)) { temp  = t; Temp10 = (int)(t * 10.0f); }
    if (!isnan(h)) { hum   = h; Hum10  = (int)(h * 10.0f); }
  }

  // --- ULTRASON (1 capteur par cycle, round-robin) ---
  if (now - lastUS >= INTERVAL_US) {
    lastUS = now;

    int d = Distance_fast(usSensors[usIndex].trig, usSensors[usIndex].echo);
    fDist[usIndex] = filterEMA(fDist[usIndex], (float)d);
    iDist[usIndex] = (int)fDist[usIndex];

    usIndex = (usIndex + 1) % US_COUNT;
  }

  // Fusion avant (prend le minimum entre AD et AG ; si l'un est à la distance max,
  // c'est qu'il ne détecte rien → on prend l'autre)
  int ad = iDist[IDX_AD];
  int ag = iDist[IDX_AG];

  if (ad >= US_MAX_DIST_CM && ag >= US_MAX_DIST_CM)
    Distance_A = US_MAX_DIST_CM;
  else if (ad >= US_MAX_DIST_CM)
    Distance_A = ag;
  else if (ag >= US_MAX_DIST_CM)
    Distance_A = ad;
  else
    Distance_A = min(ad, ag);

  // --- LUMIÈRE + BATTERIE ---
  if (now - lastLight >= INTERVAL_LIGHT) {
    lastLight = now;

    photo1Buffer[photoIndex] = analogRead(photo_1);
    photo2Buffer[photoIndex] = analogRead(photo_2);
    photo3Buffer[photoIndex] = analogRead(photo_3);
    photo4Buffer[photoIndex] = analogRead(photo_4);
    photoIndex = (photoIndex + 1) % PHOTO_SAMPLES;

    long s1=0, s2=0, s3=0, s4=0;
    for (int i = 0; i < PHOTO_SAMPLES; i++) {
      s1 += photo1Buffer[i];
      s2 += photo2Buffer[i];
      s3 += photo3Buffer[i];
      s4 += photo4Buffer[i];
    }
    val_photo_1 = s1 / PHOTO_SAMPLES;
    val_photo_2 = s2 / PHOTO_SAMPLES;
    val_photo_3 = s3 / PHOTO_SAMPLES;
    val_photo_4 = s4 / PHOTO_SAMPLES;
    val_photo_moyen = analogToLux((val_photo_1 + val_photo_2 + val_photo_3 + val_photo_4) / 4);

    // Batterie (moyenne sur 5 lectures rapides)
    long somme = 0;
    for (int i = 0; i < 5; i++) somme += analogRead(PIN_PDT);
    Ubat    = (somme / 5.0f) * (Vref / 1023.0f) * diviseur;
    Ubat100 = (int)(Ubat * 100.0f + 0.5f);
  }

  // --- GPS ---
  sat = gps.satellites.value();
  bool gpsOk = (sat >= 6 && gps.hdop.hdop() < 2.5f);

  if (gpsOk && gps.location.isValid() && gps.location.age() < 2000) {
    latE7 = (int32_t)(gps.location.lat() * 1e7);
    lngE7 = (int32_t)(gps.location.lng() * 1e7);
  }
  if (gpsOk && gps.altitude.isValid() && gps.altitude.age() < 2000) {
    alt_int = (int32_t)gps.altitude.meters();
  }
  if (gpsOk && gps.speed.isValid() && gps.speed.age() < 2000) {
    spd_int = (int32_t)(gps.speed.kmph() * 10.0f);
  }

  // --- ENVOI ---
  char buffer[128];
  sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%ld,%ld\n",
    Distance_A,
    iDist[IDX_Ar],
    iDist[IDX_D],
    iDist[IDX_G],
    val_photo_moyen, Temp10, Hum10, Ubat100,
    sat, latE7, lngE7, alt_int, spd_int
  );
  Serial2.print(buffer);
}

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial1.begin(9600);    // GPS
  Serial2.begin(115200);  // ESP

  pinMode(EN_GPS_PIN, OUTPUT);
  digitalWrite(EN_GPS_PIN, HIGH);
  delay(2000);

  Serial1.write(enableGalileo, sizeof(enableGalileo));
  delay(500);
  Serial1.write(saveConfig, sizeof(saveConfig));

  dht.begin();

  // Photorésistances
  pinMode(photo_1, INPUT);
  pinMode(photo_2, INPUT);
  pinMode(photo_3, INPUT);
  pinMode(photo_4, INPUT);

  // Ultrason
  for (int i = 0; i < US_COUNT; i++) {
    pinMode(usSensors[i].trig, OUTPUT);
    pinMode(usSensors[i].echo, INPUT);
    fDist[i] = (float)US_MAX_DIST_CM;
    iDist[i] = US_MAX_DIST_CM;
  }

  // Moteurs
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

  // Servos caméra
  ServoCamX.attach(PWM_SERVO_CAM_X);
  ServoCamX.write(SERVO_X_val);
  ServoCamY.attach(PWM_SERVO_CAM_Y);
  ServoCamY.write(SERVO_Y_val);
}

// =====================================================================
// LOOP
// =====================================================================
void loop() {
  // PRIORITÉ MAX : commandes moteurs depuis l'ESP
  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '\n') {
      serial2Buffer[serial2Index] = '\0';

      int newSX, newSY;
      int parsed = sscanf(serial2Buffer, "%d,%d,%d,%d,%d,%d",
        &AIN1_val, &AIN2_val, &BIN1_val, &BIN2_val,
        &newSX, &newSY
      );

      if (parsed == 6) {
        digitalWrite(AIN1_PIN, AIN1_val);
        digitalWrite(AIN2_PIN, AIN2_val);
        digitalWrite(BIN1_PIN, BIN1_val);
        digitalWrite(BIN2_PIN, BIN2_val);

        if (abs(newSX - SERVO_X_val) > 3) {
          ServoCamX.write(newSX);
          SERVO_X_val = newSX;
        }
        if (abs(newSY - SERVO_Y_val) > 3) {
          ServoCamY.write(newSY);
          SERVO_Y_val = newSY;
        }
      }

      serial2Index = 0;
    } else {
      if (serial2Index < sizeof(serial2Buffer) - 1)
        serial2Buffer[serial2Index++] = c;
    }
  }

  // Capteurs + envoi
  updateSensors();

  // GPS
  while (Serial1.available()) gps.encode(Serial1.read());
}
