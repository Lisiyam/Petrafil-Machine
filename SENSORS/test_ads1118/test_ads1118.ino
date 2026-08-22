#include <SPI.h>

#define ADS_CS    17
#define ADS_SCK   5
#define ADS_MISO  16
#define ADS_MOSI  4

SPISettings adsSPI(1000000, MSBFIRST, SPI_MODE1);


// =====================================================
// KONFIGURASI ADS1118
// =====================================================
//
// MUX  = AIN0 terhadap GND
// PGA  = +/- 4.096 V
// MODE = Single-shot
// DR   = 128 SPS
//
// =====================================================

const uint16_t ADS_CONFIG = 0xC383;


// =====================================================
// KONFIGURASI NTC
// =====================================================

#define VCC        3.3

// Resistor bawah
#define R_BOTTOM   10000.0       // 10k ohm

// NTC
#define NTC_R25    100000.0      // 100k ohm @ 25°C
#define NTC_T25    298.15        // 25°C = 298.15 Kelvin
#define NTC_BETA   3950.0        // Beta = 3950 K


// =====================================================
// KONFIGURASI FILTER EMA
// =====================================================
//
// EMA_ALPHA menentukan seberapa cepat filter merespons
// perubahan suhu:
//
//   - kecil (0.05 - 0.1) -> sangat halus, respons lambat
//   - besar (0.3 - 0.8)  -> responsif, kurang meredam noise
//
// Untuk suhu hotend PET (perubahan relatif lambat),
// nilai 0.1 - 0.15 biasanya cocok.
//
// =====================================================

#define EMA_ALPHA   0.12

float temperatureFiltered = NAN;   // hasil suhu setelah difilter
bool  emaInitialized      = false; // flag inisialisasi pertama kali


// =====================================================
// KONFIGURASI PWM (KONTROL HEATER VIA MOSFET IRFZ44N)
// =====================================================

#define PWM_PIN     25
const uint32_t PWM_FREQ = 5000;  // 5 kHz
const uint8_t  PWM_RES  = 8;     // 8-bit -> 0-255

int pwmValue = 0;   // nilai PWM aktif saat ini (0-255)


// =====================================================
// KALIBRASI SUHU (regresi kuadratik dari data referensi)
// =====================================================
//
// Persamaan: T_kalibrasi = A*x^2 + B*x + C
// x = suhu hasil EMA (temperatureFiltered)
//
// Didapat dari fitting 10 titik data (PWM vs suhu ref
// vs suhu terbaca sensor), R^2 = 0.9999
//
// CATATAN: valid untuk rentang ~54-232°C (rentang data
// kalibrasi). Untuk area kerja mendekati 260°C, sebaiknya
// tambahkan titik data kalibrasi baru di 250-270°C agar
// tidak ekstrapolasi terlalu jauh dari data asli.
//
// =====================================================

#define CAL_A   -0.0012123207788950524
#define CAL_B    1.1994562204279697
#define CAL_C   -6.731294736362334

float temperatureCalibrated = NAN;  // suhu akhir setelah EMA + kalibrasi


// =====================================================
// READ ADS1118
// =====================================================

uint16_t readADS1118()
{
  uint16_t raw;

  SPI.beginTransaction(adsSPI);

  digitalWrite(ADS_CS, LOW);

  raw = SPI.transfer16(ADS_CONFIG);

  digitalWrite(ADS_CS, HIGH);

  SPI.endTransaction();

  return raw;
}


// =====================================================
// KONVERSI RESISTANSI NTC -> SUHU
// =====================================================

float calculateTemperature(float resistance)
{
  if (resistance <= 0)
  {
    return NAN;
  }

  // Persamaan Beta:
  //
  // 1/T = 1/T0 + (1/B) * ln(R/R0)

  float temperatureK =
    1.0 /
    (
      (1.0 / NTC_T25) +
      (1.0 / NTC_BETA) *
      log(resistance / NTC_R25)
    );

  // Kelvin -> Celsius

  float temperatureC =
    temperatureK - 273.15;

  return temperatureC;
}


// =====================================================
// FILTER EMA UNTUK SUHU
// =====================================================

float applyEMA(float newValue)
{
  if (isnan(newValue))
  {
    // Kalau pembacaan gagal, pertahankan nilai filter
    // sebelumnya (jangan rusak filter karena satu bacaan buruk)
    return temperatureFiltered;
  }

  if (!emaInitialized)
  {
    // Inisialisasi pertama kali langsung pakai nilai asli,
    // supaya filter tidak mulai dari 0 / naik pelan dari awal
    emaInitialized = true;
    return newValue;
  }

  return (EMA_ALPHA * newValue) +
         ((1.0 - EMA_ALPHA) * temperatureFiltered);
}


// =====================================================
// KALIBRASI SUHU (KUADRATIK)
// =====================================================

float calibrateTemperature(float tRaw)
{
  if (isnan(tRaw))
  {
    return NAN;
  }

  return (CAL_A * tRaw * tRaw) +
         (CAL_B * tRaw) +
         CAL_C;
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  pinMode(ADS_CS, OUTPUT);
  digitalWrite(ADS_CS, HIGH);

  SPI.begin(
    ADS_SCK,
    ADS_MISO,
    ADS_MOSI,
    ADS_CS
  );

  delay(1000);

  Serial.println("========================================");
  Serial.println("  ADS1118 - NTC 100K B3950 (+ Filter EMA)");
  Serial.println("========================================");


  // ---------------------------------------------------
  // Inisialisasi PWM (heater control)
  // ---------------------------------------------------

  if (!ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES))
  {
    Serial.println("Gagal attach PWM!");
  }
  else
  {
    // Awal PWM = 0 (heater mati)
    ledcWrite(PWM_PIN, 0);

    Serial.println("=== PWM HEATER ===");
    Serial.println("Pin      : GPIO 25");
    Serial.println("Frequency: 5 kHz");
    Serial.println("Resolusi : 8-bit");
    Serial.println("Range    : 0-255");
    Serial.println();
    Serial.println("Masukkan nilai PWM (0-255) via Serial:");
  }
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  // ---------------------------------------------------
  // Mulai konversi
  // ---------------------------------------------------

  readADS1118();

  delay(10);


  // ---------------------------------------------------
  // Baca hasil ADC
  // ---------------------------------------------------

  uint16_t raw = readADS1118();

  int16_t signedRaw = (int16_t)raw;


  // ---------------------------------------------------
  // ADC -> VOLTAGE
  //
  // PGA = +/-4.096 V
  // ---------------------------------------------------

  float voltage =
    signedRaw * 4.096 / 32768.0;


  // ---------------------------------------------------
  // VOLTAGE -> RESISTANSI NTC
  //
  // Rangkaian:
  //
  //       3.3V
  //        |
  //      NTC
  //        |
  //        +---- A0
  //        |
  //      10k
  //        |
  //       GND
  //
  // Vout = VCC * R_BOTTOM / (R_NTC + R_BOTTOM)
  //
  // R_NTC = R_BOTTOM * (VCC/Vout - 1)
  // ---------------------------------------------------

  float ntcResistance = NAN;

  if (voltage > 0.001 && voltage < VCC)
  {
    ntcResistance =
      R_BOTTOM *
      ((VCC / voltage) - 1.0);
  }


  // ---------------------------------------------------
  // RESISTANSI NTC -> SUHU (mentah / belum difilter)
  // ---------------------------------------------------

  float temperatureC = NAN;

  if (!isnan(ntcResistance))
  {
    temperatureC =
      calculateTemperature(ntcResistance);
  }


  // ---------------------------------------------------
  // FILTER EMA
  // ---------------------------------------------------

  temperatureFiltered = applyEMA(temperatureC);


  // ---------------------------------------------------
  // KALIBRASI (koreksi kuadratik)
  // ---------------------------------------------------

  temperatureCalibrated = calibrateTemperature(temperatureFiltered);


  // ---------------------------------------------------
  // SERIAL OUTPUT
  // ---------------------------------------------------

  Serial.print("RAW = ");
  Serial.print(signedRaw);

  Serial.print("    Voltage = ");
  Serial.print(voltage, 6);
  Serial.print(" V");

  Serial.print("    R_NTC = ");
  Serial.print(ntcResistance, 2);
  Serial.print(" Ohm");

  Serial.print("    Temp Raw = ");
  Serial.print(temperatureC, 2);
  Serial.print(" °C");

  Serial.print("    Temp EMA = ");
  Serial.print(temperatureFiltered, 2);
  Serial.print(" °C");

  Serial.print("    Temp Kalibrasi = ");
  Serial.print(temperatureCalibrated, 2);
  Serial.println(" °C");


  // ---------------------------------------------------
  // KONTROL PWM DARI SERIAL
  // ---------------------------------------------------

  if (Serial.available() > 0)
  {
    int inputValue = Serial.parseInt();

    // Pastikan nilai berada dalam range 0-255
    pwmValue = constrain(inputValue, 0, 255);

    // Set PWM
    ledcWrite(PWM_PIN, pwmValue);

    Serial.print("PWM = ");
    Serial.print(pwmValue);
    Serial.print(" | Duty Cycle = ");
    Serial.print((pwmValue / 255.0) * 100.0, 1);
    Serial.println("%");

    // Bersihkan sisa karakter newline
    while (Serial.available())
    {
      Serial.read();
    }
  }


  delay(500);
}
