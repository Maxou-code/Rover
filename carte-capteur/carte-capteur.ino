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
  
  // Attente de la première position GPS valide
  while (!gps.location.isValid()) {
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
  }

  digitalWrite(led_on, HIGH);
}

void loop() {
  Distance_AD = Distance_test(Trig_Capteur_US_AD, Echo_Capteur_US_AD);
  Distance_Ar = Distance_test(Trig_Capteur_US_Ar, Echo_Capteur_US_Ar);
  Distance_D = Distance_test(Trig_Capteur_US_D, Echo_Capteur_US_D);
  Distance_G = Distance_test(Trig_Capteur_US_G, Echo_Capteur_US_G);
  Distance_AG = Distance_test(Trig_Capteur_US_AG, Echo_Capteur_US_AG);

  Distance_A = (Distance_AD < Distance_AG) ? Distance_AD : Distance_AG;

  Serial.print(Distance_A);
  Serial.print(",");
  Serial.print(Distance_Ar);
  Serial.print(",");
  Serial.print(Distance_D);
  Serial.print(",");
  Serial.print(Distance_G);
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
