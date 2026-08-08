#define STEP_PIN 26
#define DIR_PIN 27
#define EN_PIN 14

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);    // Aktifkan driver
  digitalWrite(DIR_PIN, HIGH);  // Searah jarum jam
}

void loop() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(800);

  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(800);
}
//coba aja