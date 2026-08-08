#include <SPI.h>

#define ADS_CS    5
#define ADS_SCK   18
#define ADS_MISO  19
#define ADS_MOSI  23

SPISettings adsSPI(1000000, MSBFIRST, SPI_MODE1);


// Konfigurasi ADS1118:
//
// MUX  = AIN0 terhadap GND
// PGA  = +/- 4.096 V
// MODE = Single-shot
// DR   = 128 SPS
//
// 0xC383
//
const uint16_t ADS_CONFIG = 0xC383;


uint16_t readADS1118()
{
  uint16_t raw;

  SPI.beginTransaction(adsSPI);

  digitalWrite(ADS_CS, LOW);

  // Baca hasil konversi sebelumnya
  // sekaligus memulai konversi berikutnya
  raw = SPI.transfer16(ADS_CONFIG);

  digitalWrite(ADS_CS, HIGH);

  SPI.endTransaction();

  return raw;
}


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

  Serial.println("================================");
  Serial.println("      ADS1118 TEST - A0");
  Serial.println("================================");
}


void loop()
{
  // Mulai konversi pertama
  readADS1118();

  // 128 SPS -> sekitar 7.8 ms per conversion
  delay(10);

  // Baca hasil konversi
  uint16_t raw = readADS1118();

  // ADS1118 menghasilkan data two's complement.
  int16_t signedRaw = (int16_t)raw;

  // PGA +/-4.096 V
  // 1 LSB = 4.096 / 32768 V
  float voltage = signedRaw * 4.096 / 32768.0;

  Serial.print("RAW = ");
  Serial.print(signedRaw);

  Serial.print("    Voltage = ");
  Serial.print(voltage, 6);

  Serial.println(" V");

  delay(500);
}