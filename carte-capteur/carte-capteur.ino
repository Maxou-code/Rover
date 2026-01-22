#include <DHT.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include <Arduino.h>
#include <math.h>

TinyGPSPlus gps;

Servo ServoCam;

#define AIN1_PIN 33
#define AIN2_PIN 27

#define BIN1_PIN 39
#define BIN2_PIN 45

#define PWM_SERVO_CAM 7

#define DHT_PIN 2
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);
float temp, hum;

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

double lat, lng;

int led_on = 4;

int AIN1_val = 0;
int AIN2_val = 0;

int BIN1_val = 0;
int BIN2_val = 0;

int SERVO_val = 90;

unsigned long lastSensorUpdate = 0;
const unsigned long SENSOR_INTERVAL = 500;  // ms

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

int Distance_test(int trig, int echo) {
    const int N = 5; // nombre de mesures
    int mesures[N];

    // prendre N mesures
    for (int i = 0; i < N; i++) {
        digitalWrite(trig, LOW);
        delayMicroseconds(2);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);
        long duration = pulseIn(echo, HIGH, 25000);
        if (duration == 0) mesures[i] = -1; 
        else mesures[i] = duration / 58;
        delay(5);
    }

    // compter les valeurs valides
    int valid[N];
    int count = 0;
    for (int i = 0; i < N; i++) {
        if (mesures[i] != -1) {
            valid[count++] = mesures[i];
        }
    }

    if (count == 0) return -1; // aucune mesure valide

    // tri simple à bulles
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            if (valid[j] > valid[j + 1]) {
                int tmp = valid[j];
                valid[j] = valid[j + 1];
                valid[j + 1] = tmp;
            }
        }
    }

    // retourner la médiane
    if (count % 2 == 1) {
        return valid[count / 2];
    } else {
        return (valid[count / 2 - 1] + valid[count / 2]) / 2;
    }
}

void setup() {
  Serial1.begin(9600);    // GPS
  Serial2.begin(115200);  // ESP

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

  pinMode(led_on, OUTPUT);

  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);

  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN2_PIN, LOW);

  ServoCam.attach(PWM_SERVO_CAM);
  ServoCam.write(SERVO_val);

  // Attente de la première position GPS valide
  while (!gps.location.isValid()) {
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
  }

  digitalWrite(led_on, HIGH);
}

void loop() {

  // Lecture Serial2 (UART ESP)
  while (Serial2.available()) {
    char c = Serial2.read();

    // Fin de trame
    if (c == '\n') {
      serial2Buffer[serial2Index] = '\0'; // Fin de string

      // Parsing : AIN1, AIN2, BIN1, BIN2, SERVO
      int parsed = sscanf(
        serial2Buffer,
        "%d,%d,%d,%d,%d",
        &AIN1_val,
        &AIN2_val,
        &BIN1_val,
        &BIN2_val,
        &SERVO_val
      );

      if (parsed == 5) {
        updateMotors();
      }

      // Reset buffer
      serial2Index = 0;
    }
    else {
      // Stockage caractère si buffer pas plein
      if (serial2Index < sizeof(serial2Buffer) - 1) {
        serial2Buffer[serial2Index++] = c;
      }
    }
  }

  // Mise à jour des capteurs (inchangé)
  if (millis() - lastSensorUpdate >= SENSOR_INTERVAL) {
    lastSensorUpdate = millis();
    updateSensors();
  }
}

void updateMotors() {
  digitalWrite(AIN1_PIN, AIN1_val);
  digitalWrite(AIN2_PIN, AIN2_val);
  digitalWrite(BIN1_PIN, BIN1_val);
  digitalWrite(BIN2_PIN, BIN2_val);

  ServoCam.write(SERVO_val);
}

void updateSensors() {
  unsigned long currentMillis = millis();

  // Lire le capteur suivant si 50 ms se sont écoulées
  if (currentMillis - lastSensorTime >= sensorInterval) {
    lastSensorTime = currentMillis;

    switch(sensorIndex) {
      case 0:
        Distance_AD = Distance_test(Trig_Capteur_US_AD, Echo_Capteur_US_AD);
        break;
      case 1:
        Distance_Ar = Distance_test(Trig_Capteur_US_Ar, Echo_Capteur_US_Ar);
        break;
      case 2:
        Distance_D = Distance_test(Trig_Capteur_US_D, Echo_Capteur_US_D);
        break;
      case 3:
        Distance_G = Distance_test(Trig_Capteur_US_G, Echo_Capteur_US_G);
        break;
      case 4:
        Distance_AG = Distance_test(Trig_Capteur_US_AG, Echo_Capteur_US_AG);
        break;
    }

    sensorIndex++;
    if (sensorIndex > 4) sensorIndex = 0; // Repartir du début
  }

  // Calcul de Distance_A
  Distance_A =
      (Distance_AD == -1) ? Distance_AG :
      (Distance_AG == -1) ? Distance_AD :
      (Distance_AD < Distance_AG ? Distance_AD : Distance_AG);

  // --- Lecture des photos avec moyenne glissante ---
  photo1Buffer[photoIndex] = analogRead(photo_1);
  photo2Buffer[photoIndex] = analogRead(photo_2);
  photo3Buffer[photoIndex] = analogRead(photo_3);
  photo4Buffer[photoIndex] = analogRead(photo_4);

  photoIndex = (photoIndex + 1) % PHOTO_SAMPLES; // passer à l'indice suivant

  // Calcul des moyennes
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

  // Calcul de la valeur moyenne des 4 LDR
  val_photo_moyen = (val_photo_1 + val_photo_2 + val_photo_3 + val_photo_4) / 4;

  // Conversion en lux
  val_photo_moyen = analogToLux(val_photo_moyen);

  // Lecture DHT
  temp = dht.readTemperature();
  hum = dht.readHumidity();

  // GPS
  while (Serial1.available()) {
    gps.encode(Serial1.read());
    if (gps.location.isUpdated()) {
      lat = gps.location.lat();
      lng = gps.location.lng();
    }
  }

  // Envoi des données
  Serial2.print(Distance_A);
  Serial2.print(",");
  Serial2.print(Distance_Ar);
  Serial2.print(",");
  Serial2.print(Distance_D);
  Serial2.print(",");
  Serial2.print(Distance_G);
  Serial2.print(",");
  Serial2.print(val_photo_moyen);
  Serial2.print(",");
  Serial2.print(temp, 0);
  Serial2.print(",");
  Serial2.print(hum, 0);
  Serial2.print(",");
  printDMS(lat, true);
  Serial2.print(",");
  printDMS(lng, false);
  Serial2.println();
}

void printDMS(double coord, bool isLatitude) {
  char direction = (isLatitude ? (coord >= 0 ? 'N' : 'S') : (coord >= 0 ? 'E' : 'W'));
  coord = abs(coord);

  int degrees = int(coord);
  double minutesDecimal = (coord - degrees) * 60;
  int minutes = int(minutesDecimal);
  double seconds = (minutesDecimal - minutes) * 60;

  Serial2.print(degrees);
  Serial2.print("°");
  Serial2.print(minutes);
  Serial2.print("'");
  Serial2.print(seconds, 2);
  Serial2.print("\"");
  Serial2.print(direction);
}