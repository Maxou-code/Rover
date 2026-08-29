// JOYSTICK AVEC MOYENNE GLISSANTE

#define PIN_X A0
#define PIN_Y A1
#define PIN_SW 7

#define NUM_SAMPLES 20  // Nombre de mesures pour la moyenne

int Val_X;
int Val_Y;
int Val_SW;

int bufferX[NUM_SAMPLES];
int bufferY[NUM_SAMPLES];
int indexSample = 0;

void setup() {
  Serial.begin(9600);

  pinMode(PIN_X, INPUT);
  pinMode(PIN_Y, INPUT);
  pinMode(PIN_SW, INPUT_PULLUP);

  // Initialisation des buffers
  for(int i = 0; i < NUM_SAMPLES; i++) {
    bufferX[i] = analogRead(PIN_X);
    bufferY[i] = analogRead(PIN_Y);
  }
}

int moyenne(int* buffer) {
  long sum = 0;
  for(int i = 0; i < NUM_SAMPLES; i++) {
    sum += buffer[i];
  }
  return sum / NUM_SAMPLES;
}

void loop() {
  // Lire les valeurs actuelles
  bufferX[indexSample] = analogRead(PIN_X);
  bufferY[indexSample] = analogRead(PIN_Y);
  indexSample = (indexSample + 1) % NUM_SAMPLES;

  // Calculer la moyenne
  Val_X = moyenne(bufferX);
  Val_Y = moyenne(bufferY);
  Val_SW = digitalRead(PIN_SW);

  // Affichage
  Serial.print(Val_X);
  Serial.print(",");
  Serial.print(Val_Y);
  Serial.print(",");
  Serial.println(Val_SW);

  delay(50); // Petit délai pour stabilité
}