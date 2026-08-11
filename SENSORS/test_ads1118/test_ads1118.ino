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
  Serial.println("       ADS1118 - NTC 100K B3950");
  Serial.println("========================================");
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
  // RESISTANSI NTC -> SUHU
  // ---------------------------------------------------

  float temperatureC = NAN;

  if (!isnan(ntcResistance))
  {
    temperatureC =
      calculateTemperature(ntcResistance);
  }


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

  Serial.print("    Temperature = ");
  Serial.print(temperatureC, 2);
  Serial.println(" °C");


  delay(500);
}