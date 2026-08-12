#define PWM_PIN 25

const uint32_t PWM_FREQ = 5000;  // 5 kHz
const uint8_t PWM_RES = 8;       // 8-bit -> 0-255

void setup() {
  Serial.begin(115200);

  // Attach PWM ke GPIO 25
  if (!ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES)) {
    Serial.println("Gagal attach PWM!");
    return;
  }

  // Awal PWM = 0
  ledcWrite(PWM_PIN, 0);

  Serial.println("=== PWM TEST ESP32 ===");
  Serial.println("Pin      : GPIO 25");
  Serial.println("Frequency: 5 kHz");
  Serial.println("Resolusi : 8-bit");
  Serial.println("Range    : 0-255");
  Serial.println();
  Serial.println("Masukkan nilai PWM:");
}

void loop() {
  if (Serial.available() > 0) {

    int pwmValue = Serial.parseInt();

    // Pastikan nilai berada dalam range 0-255
    pwmValue = constrain(pwmValue, 0, 255);

    // Set PWM
    ledcWrite(PWM_PIN, pwmValue);

    Serial.print("PWM = ");
    Serial.print(pwmValue);

    Serial.print(" | Duty Cycle = ");
    Serial.print((pwmValue / 255.0) * 100.0, 1);
    Serial.println("%");

    // Bersihkan sisa karakter newline
    while (Serial.available()) {
      Serial.read();
    }
  }
}