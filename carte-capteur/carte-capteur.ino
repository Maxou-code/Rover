#include <DHT.h>
#include <TinyGPS++.h>
#include <Servo.h>

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

int Distance_test(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 25000);  // ~4m max
  if (duration == 0) return -1;
  return duration / 58;
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
  Distance_AD = Distance_test(Trig_Capteur_US_AD, Echo_Capteur_US_AD);
  Distance_Ar = Distance_test(Trig_Capteur_US_Ar, Echo_Capteur_US_Ar);
  Distance_D = Distance_test(Trig_Capteur_US_D, Echo_Capteur_US_D);
  Distance_G = Distance_test(Trig_Capteur_US_G, Echo_Capteur_US_G);
  Distance_AG = Distance_test(Trig_Capteur_US_AG, Echo_Capteur_US_AG);

  Distance_A = (Distance_AD < Distance_AG) ? Distance_AD : Distance_AG;

  Serial2.print(Distance_A);
  Serial2.print(",");
  Serial2.print(Distance_Ar);
  Serial2.print(",");
  Serial2.print(Distance_D);
  Serial2.print(",");
  Serial2.print(Distance_G);
  Serial2.print(",");

  val_photo_1 = analogRead(photo_1);
  val_photo_2 = analogRead(photo_2);
  val_photo_3 = analogRead(photo_3);
  val_photo_4 = analogRead(photo_4);
  val_photo_moyen = (val_photo_1 + val_photo_2 + val_photo_3 + val_photo_4) / 4;
  Serial2.print(val_photo_moyen);
  Serial2.print(",");

  temp = dht.readTemperature();
  hum = dht.readHumidity();

  Serial2.print(temp, 0);
  Serial2.print(",");
  Serial2.print(hum, 0);
  Serial2.print(",");

  while (Serial1.available()) {
    gps.encode(Serial1.read());  // Traitement des données GPS

    if (gps.location.isUpdated()) {
      lat = gps.location.lat();
      lng = gps.location.lng();
    }
  }

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