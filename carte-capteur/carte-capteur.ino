#include <DHT.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;

#define DHT_PIN 2
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);
float temp, hum;

int photo_1 = A0;
int photo_2 = A1;
int photo_3 = A2;
int photo_4 = A3;

int val_photo_1, val_photo_2, val_photo_3, val_photo_4, val_photo_moyen;

int Echo_1 = 24;
int Trig_1 = 22;

int Echo_2 = 30;
int Trig_2 = 28;

int Echo_3 = 36;
int Trig_3 = 34;

int Echo_4 = 42;
int Trig_4 = 40;

int Echo_5 = 48;
int Trig_5 = 46;

int Distance_1, Distance_2, Distance_3, Distance_4, Distance_5;

int Distance_avant;

double lat, lng;

int led_on = 4;

int Distance_test(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  return pulseIn(echo, HIGH) / 58;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  
  dht.begin();

  pinMode(photo_1, INPUT);
  pinMode(photo_2, INPUT);
  pinMode(photo_3, INPUT);
  pinMode(photo_4, INPUT);

  pinMode(Echo_1, INPUT);
  pinMode(Trig_1, OUTPUT);
  pinMode(Echo_2, INPUT);
  pinMode(Trig_2, OUTPUT);
  pinMode(Echo_3, INPUT);
  pinMode(Trig_3, OUTPUT);
  pinMode(Echo_4, INPUT);
  pinMode(Trig_4, OUTPUT);
  pinMode(Echo_5, INPUT);
  pinMode(Trig_5, OUTPUT);

  pinMode(led_on, OUTPUT);
  
  // Attente de la première position GPS valide
  while (!gps.location.isValid()) {
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
  }

  digitalWrite(led_on, HIGH);
}

void loop() {
  Distance_1 = Distance_test(Trig_1, Echo_1);
  Distance_2 = Distance_test(Trig_2, Echo_2);
  Distance_3 = Distance_test(Trig_3, Echo_3);
  Distance_4 = Distance_test(Trig_4, Echo_4);
  Distance_5 = Distance_test(Trig_5, Echo_5);

  Distance_avant = (Distance_1 < Distance_5) ? Distance_1 : Distance_5;

  Serial.print(Distance_avant);
  Serial.print(",");
  Serial.print(Distance_2);
  Serial.print(",");
  Serial.print(Distance_3);
  Serial.print(",");
  Serial.print(Distance_4);
  Serial.print(",");

  val_photo_1 = analogRead(photo_1);
  val_photo_2 = analogRead(photo_2);
  val_photo_3 = analogRead(photo_3);
  val_photo_4 = analogRead(photo_4);
  val_photo_moyen = (val_photo_1 + val_photo_2 + val_photo_3 + val_photo_4) / 4;
  Serial.print(val_photo_moyen);
  Serial.print(",");

  temp = dht.readTemperature();
  hum = dht.readHumidity();

  Serial.print(temp, 0);
  Serial.print(",");
  Serial.print(hum, 0);
  Serial.print(",");

  if (Serial1.available() > 0) {
      gps.encode(Serial1.read());  // Traitement des données GPS

      if (gps.location.isUpdated()) {
          lat = gps.location.lat();
          lng = gps.location.lng();
      }
  }

  printDMS(lat, true);
  Serial.print(",");
  printDMS(lng, false);
  Serial.println();

  // delay(1000);
}

void printDMS(double coord, bool isLatitude) {
    char direction = (isLatitude ? (coord >= 0 ? 'N' : 'S') : (coord >= 0 ? 'E' : 'W'));
    coord = abs(coord);
    
    int degrees = int(coord);
    double minutesDecimal = (coord - degrees) * 60;
    int minutes = int(minutesDecimal);
    double seconds = (minutesDecimal - minutes) * 60;

    Serial.print(degrees);
    Serial.print("°");
    Serial.print(minutes);
    Serial.print("'");
    Serial.print(seconds, 2);
    Serial.print("\"");
    Serial.print(direction);
}
