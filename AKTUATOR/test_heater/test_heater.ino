#include <SPI.h>
#include <PID_v1.h>


// =====================================================
// PIN ADS1118
// =====================================================

#define ADS_CS    17
#define ADS_SCK   5
#define ADS_MISO  16
#define ADS_MOSI  4


// =====================================================
// PIN HEATER PWM
// =====================================================

#define HEATER_PIN 25

#define PWM_FREQ 5000
#define PWM_RES  8


// =====================================================
// SPI ADS1118
// =====================================================

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

#define VCC       3.3
#define R_BOTTOM  10000.0
#define NTC_R25   100000.0
#define NTC_T25   298.15
#define NTC_BETA  3950.0


// =====================================================
// PID
// =====================================================

double temperatureC = 0.0;

double setpoint = 260.0;

double pidOutput = 0.0;


// Nilai awal PID
//
// Ini hanya starting point.
// Hotend sebenarnya harus dituning karena setiap
// heater, nozzle, blok heater, dan insulation berbeda.

double Kp = 8.0;
double Ki = 0.4;
double Kd = 30.0;


// DIRECT:
// suhu turun -> output heater naik
// suhu naik   -> output heater turun

PID hotendPID(
  &temperatureC,
  &pidOutput,
  &setpoint,
  Kp,
  Ki,
  Kd,
  DIRECT
);


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

  float temperatureK =
    1.0 /
    (
      (1.0 / NTC_T25) +
      (1.0 / NTC_BETA) *
      log(resistance / NTC_R25)
    );

  float temperatureC =
    temperatureK - 273.15;

  return temperatureC;
}


// =====================================================
// BACA SUHU
// =====================================================

float readTemperature()
{
  // Mulai conversion
  readADS1118();

  // ADS1118 128 SPS membutuhkan waktu sekitar 7.8 ms
  delay(10);

  // Ambil hasil conversion
  uint16_t raw = readADS1118();

  int16_t signedRaw = (int16_t)raw;


  // ---------------------------------------------------
  // ADC -> VOLTAGE
  // PGA +/-4.096V
  // ---------------------------------------------------

  float voltage =
    signedRaw * 4.096 / 32768.0;


  // ---------------------------------------------------
  // VOLTAGE -> RESISTANSI NTC
  // ---------------------------------------------------

  float ntcResistance = NAN;

  if (voltage > 0.001 && voltage < VCC)
  {
    ntcResistance =
      R_BOTTOM *
      ((VCC / voltage) - 1.0);
  }


  // ---------------------------------------------------
  // RESISTANSI -> TEMPERATURE
  // ---------------------------------------------------

  float temp = NAN;

  if (!isnan(ntcResistance))
  {
    temp =
      calculateTemperature(ntcResistance);
  }


  // ---------------------------------------------------
  // DEBUG
  // ---------------------------------------------------

  Serial.print("RAW = ");
  Serial.print(signedRaw);

  Serial.print(" | Voltage = ");
  Serial.print(voltage, 6);
  Serial.print(" V");

  Serial.print(" | R_NTC = ");
  Serial.print(ntcResistance, 2);
  Serial.print(" Ohm");

  Serial.print(" | Temperature = ");
  Serial.print(temp, 2);
  Serial.println(" C");


  return temp;
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);


  // ---------------------------------------------------
  // ADS1118
  // ---------------------------------------------------

  pinMode(ADS_CS, OUTPUT);
  digitalWrite(ADS_CS, HIGH);

  SPI.begin(
    ADS_SCK,
    ADS_MISO,
    ADS_MOSI,
    ADS_CS
  );


  // ---------------------------------------------------
  // PWM HEATER
  // ---------------------------------------------------

  if (!ledcAttach(HEATER_PIN, PWM_FREQ, PWM_RES))
  {
    Serial.println("ERROR: PWM attach gagal!");
    while (1);
  }

  // Heater OFF saat startup
  ledcWrite(HEATER_PIN, 0);


  // ---------------------------------------------------
  // PID
  // ---------------------------------------------------

  hotendPID.SetOutputLimits(0, 255);

  // PID dihitung setiap 500 ms
  hotendPID.SetSampleTime(500);

  hotendPID.SetMode(AUTOMATIC);


  // ---------------------------------------------------
  // INFO
  // ---------------------------------------------------

  Serial.println();
  Serial.println("========================================");
  Serial.println("       HOTEND PID CONTROLLER");
  Serial.println("========================================");

  Serial.println("Setpoint = 260 C");

  Serial.print("Kp = ");
  Serial.println(Kp);

  Serial.print("Ki = ");
  Serial.println(Ki);

  Serial.print("Kd = ");
  Serial.println(Kd);

  Serial.println();

  delay(1000);
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  // ---------------------------------------------------
  // BACA SUHU
  // ---------------------------------------------------

  float newTemperature = readTemperature();


  // Pastikan pembacaan valid
  if (!isnan(newTemperature))
  {
    temperatureC = newTemperature;
  }
  else
  {
    // Jika sensor bermasalah,
    // MATIKAN HEATER
    pidOutput = 0;

    ledcWrite(HEATER_PIN, 0);

    Serial.println("ERROR NTC! HEATER OFF!");

    delay(500);

    return;
  }


  // ---------------------------------------------------
  // HITUNG PID
  // ---------------------------------------------------

  hotendPID.Compute();


  // ---------------------------------------------------
  // BATASI OUTPUT
  // ---------------------------------------------------

  pidOutput = constrain(pidOutput, 0, 255);


  // ---------------------------------------------------
  // OUTPUT PWM
  // ---------------------------------------------------

  ledcWrite(
    HEATER_PIN,
    (uint32_t)pidOutput
  );


  // ---------------------------------------------------
  // DEBUG PID
  // ---------------------------------------------------

  Serial.print("TEMP = ");
  Serial.print(temperatureC, 2);

  Serial.print(" C | SETPOINT = ");
  Serial.print(setpoint, 2);

  Serial.print(" C | PID = ");
  Serial.print(pidOutput, 2);

  Serial.print(" | PWM = ");
  Serial.print((int)pidOutput);

  Serial.print(" | DUTY = ");
  Serial.print((pidOutput / 255.0) * 100.0, 1);

  Serial.println("%");


  delay(5000);
}