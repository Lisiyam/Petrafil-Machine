#include <SPI.h>
#include <PID_v1.h>
#include <math.h>


// =====================================================
// PIN ADS1118
// =====================================================

#define ADS_CS    17
#define ADS_SCK   5
#define ADS_MISO  16
#define ADS_MOSI  4


// =====================================================
// PIN HEATER
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
// SETPOINT & SAFETY
// =====================================================

double setpoint = 260.0;

const double MAX_TEMP = 280.0;


// =====================================================
// PID
// =====================================================

double temperatureC = 0.0;
double pidOutput = 0.0;

// Nilai awal sebelum autotuning.
// Setelah autotuning selesai, nilai ini akan diganti.
double Kp = 8.0;
double Ki = 0.4;
double Kd = 30.0;

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
// MODE SISTEM
// =====================================================

enum SystemMode
{
  MODE_PID,
  MODE_AUTOTUNE_WARMUP,
  MODE_AUTOTUNE_RELAY,
  MODE_STOPPED,
  MODE_FAULT
};

SystemMode mode = MODE_PID;


// =====================================================
// AUTOTUNE
// =====================================================
//
// Relay autotuning:
//
// HIGH output = 210
// LOW output  = 30
//
// Relay amplitude d:
//
// d = (210 - 30) / 2 = 90
//
// Hysteresis = +/- 2°C
//
// =====================================================

const double AT_HIGH_OUTPUT = 210.0;
const double AT_LOW_OUTPUT  = 30.0;

const double AT_RELAY_AMPLITUDE =
  (AT_HIGH_OUTPUT - AT_LOW_OUTPUT) / 2.0;

const double AT_HYSTERESIS = 2.0;

const int AT_REQUIRED_PEAKS = 6;
const int AT_REQUIRED_TROUGHS = 6;


// Penyimpanan hasil autotune
double peakValues[8];
double troughValues[8];

unsigned long peakTimes[8];

int peakCount = 0;
int troughCount = 0;

bool relayHigh = true;

unsigned long autotuneStartTime = 0;
unsigned long lastPeakTime = 0;


// =====================================================
// PWM
// =====================================================

void setHeaterPWM(uint8_t pwm)
{
  ledcWrite(HEATER_PIN, pwm);
}


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

  return temperatureK - 273.15;
}


// =====================================================
// BACA TEMPERATUR
// =====================================================

float readTemperature()
{
  // Mulai conversion
  readADS1118();

  delay(10);

  // Ambil hasil conversion
  uint16_t raw = readADS1118();

  int16_t signedRaw = (int16_t)raw;


  // ADC -> voltage

  float voltage =
    signedRaw * 4.096 / 32768.0;


  // Voltage -> resistance

  float ntcResistance = NAN;

  if (voltage > 0.001 && voltage < VCC)
  {
    ntcResistance =
      R_BOTTOM *
      ((VCC / voltage) - 1.0);
  }


  // Resistance -> temperature

  if (!isnan(ntcResistance))
  {
    return calculateTemperature(ntcResistance);
  }

  return NAN;
}


// =====================================================
// RESET AUTOTUNE DATA
// =====================================================

void resetAutotuneData()
{
  peakCount = 0;
  troughCount = 0;

  lastPeakTime = 0;

  for (int i = 0; i < 8; i++)
  {
    peakValues[i] = 0;
    troughValues[i] = 0;
    peakTimes[i] = 0;
  }
}


// =====================================================
// MULAI AUTOTUNE
// =====================================================

void startAutotune()
{
  Serial.println();
  Serial.println("========================================");
  Serial.println("         AUTOTUNE DIMULAI");
  Serial.println("========================================");

  Serial.println("Pastikan nozzle/hotend aman.");
  Serial.println("Jangan memasukkan PET selama autotuning.");
  Serial.println();

  resetAutotuneData();

  autotuneStartTime = millis();

  hotendPID.SetMode(MANUAL);

  pidOutput = 0;

  // Masuk fase pemanasan awal
  mode = MODE_AUTOTUNE_WARMUP;

  setHeaterPWM(180);

  Serial.println("Heating menuju area autotuning...");
}


// =====================================================
// MULAI RELAY AUTOTUNE
// =====================================================

void startRelayAutotune()
{
  Serial.println();
  Serial.println("========================================");
  Serial.println("       RELAY AUTOTUNE DIMULAI");
  Serial.println("========================================");

  Serial.print("Setpoint      : ");
  Serial.print(setpoint);
  Serial.println(" C");

  Serial.print("Output HIGH   : ");
  Serial.println(AT_HIGH_OUTPUT);

  Serial.print("Output LOW    : ");
  Serial.println(AT_LOW_OUTPUT);

  Serial.print("Hysteresis    : +/- ");
  Serial.print(AT_HYSTERESIS);
  Serial.println(" C");

  Serial.println();


  resetAutotuneData();


  // Mulai dari HIGH
  relayHigh = true;

  setHeaterPWM((uint8_t)AT_HIGH_OUTPUT);

  mode = MODE_AUTOTUNE_RELAY;
}


// =====================================================
// SELESAIKAN AUTOTUNE
// =====================================================

void finishAutotune()
{
  setHeaterPWM(0);

  Serial.println();
  Serial.println("========================================");
  Serial.println("        AUTOTUNE SELESAI");
  Serial.println("========================================");


  // ---------------------------------------------------
  // Hitung rata-rata peak
  // ---------------------------------------------------

  double peakAverage = 0;

  for (int i = 0; i < peakCount; i++)
  {
    peakAverage += peakValues[i];
  }

  peakAverage /= peakCount;


  // ---------------------------------------------------
  // Hitung rata-rata trough
  // ---------------------------------------------------

  double troughAverage = 0;

  for (int i = 0; i < troughCount; i++)
  {
    troughAverage += troughValues[i];
  }

  troughAverage /= troughCount;


  // ---------------------------------------------------
  // Amplitudo osilasi
  // ---------------------------------------------------

  double amplitude =
    (peakAverage - troughAverage) / 2.0;


  // ---------------------------------------------------
  // Hitung periode rata-rata
  // berdasarkan peak-to-peak
  // ---------------------------------------------------

  double periodSum = 0;
  int periodCount = 0;

  for (int i = 1; i < peakCount; i++)
  {
    double period =
      (peakTimes[i] - peakTimes[i - 1]) / 1000.0;

    periodSum += period;
    periodCount++;
  }


  if (amplitude <= 0 || periodCount <= 0)
  {
    Serial.println("Autotuning gagal.");
    Serial.println("Osilasi tidak valid.");

    mode = MODE_STOPPED;

    Serial.println();
    Serial.println("Heater OFF.");

    return;
  }


  double Pu =
    periodSum / periodCount;


  // ---------------------------------------------------
  // Ultimate gain Ku
  //
  // Ku = 4d / (pi*a)
  //
  // d = relay amplitude
  // a = amplitude suhu
  // ---------------------------------------------------

  double Ku =
    (4.0 * AT_RELAY_AMPLITUDE) /
    (PI * amplitude);


  // ---------------------------------------------------
  // Ziegler-Nichols PID
  // ---------------------------------------------------

  double newKp =
    0.6 * Ku;


  double Ti =
    0.5 * Pu;


  double Td =
    0.125 * Pu;


  double newKi =
    newKp / Ti;


  double newKd =
    newKp * Td;


  // ---------------------------------------------------
  // Simpan parameter
  // ---------------------------------------------------

  Kp = newKp;
  Ki = newKi;
  Kd = newKd;


  // Update PID
  hotendPID.SetTunings(Kp, Ki, Kd);


  // ---------------------------------------------------
  // Tampilkan hasil
  // ---------------------------------------------------

  Serial.println();

  Serial.print("Peak average   = ");
  Serial.print(peakAverage, 3);
  Serial.println(" C");

  Serial.print("Trough average = ");
  Serial.print(troughAverage, 3);
  Serial.println(" C");

  Serial.print("Amplitude       = ");
  Serial.print(amplitude, 3);
  Serial.println(" C");

  Serial.print("Pu              = ");
  Serial.print(Pu, 3);
  Serial.println(" s");

  Serial.print("Ku              = ");
  Serial.println(Ku, 5);

  Serial.println();

  Serial.println("PID hasil autotune:");

  Serial.print("Kp = ");
  Serial.println(Kp, 6);

  Serial.print("Ki = ");
  Serial.println(Ki, 6);

  Serial.print("Kd = ");
  Serial.println(Kd, 6);

  Serial.println();

  // ---------------------------------------------------
  // Setelah autotune langsung masuk PID
  // ---------------------------------------------------

  pidOutput = 0;

  hotendPID.SetMode(AUTOMATIC);

  mode = MODE_PID;

  Serial.println("Masuk ke mode PID normal.");
  Serial.println();
}


// =====================================================
// PROSES AUTOTUNE WARMUP
// =====================================================

void processAutotuneWarmup()
{
  // Pemanasan menggunakan PWM 180

  setHeaterPWM(180);


  // Sudah mendekati setpoint?
  if (temperatureC >= setpoint - 10.0)
  {
    Serial.println();
    Serial.println("Temperatur sudah mendekati setpoint.");

    setHeaterPWM(AT_HIGH_OUTPUT);

    startRelayAutotune();
  }
}


// =====================================================
// PROSES RELAY AUTOTUNE
// =====================================================

void processRelayAutotune()
{
  unsigned long now = millis();


  // ---------------------------------------------------
  // Temperatur sudah terlalu tinggi
  // ---------------------------------------------------

  if (temperatureC >= MAX_TEMP)
  {
    setHeaterPWM(0);

    Serial.println();
    Serial.println("AUTOTUNE ABORT!");
    Serial.println("Overtemperature.");

    mode = MODE_FAULT;

    return;
  }


  // ---------------------------------------------------
  // RELAY HIGH
  // ---------------------------------------------------

  if (relayHigh)
  {
    setHeaterPWM((uint8_t)AT_HIGH_OUTPUT);


    // Jika mencapai batas atas
    if (temperatureC >= setpoint + AT_HYSTERESIS)
    {
      // Simpan peak
      if (peakCount < 8)
      {
        peakValues[peakCount] = temperatureC;
        peakTimes[peakCount] = now;

        peakCount++;
      }


      Serial.print("PEAK #");
      Serial.print(peakCount);
      Serial.print(" = ");
      Serial.print(temperatureC, 2);
      Serial.println(" C");


      relayHigh = false;

      setHeaterPWM((uint8_t)AT_LOW_OUTPUT);
    }
  }


  // ---------------------------------------------------
  // RELAY LOW
  // ---------------------------------------------------

  else
  {
    setHeaterPWM((uint8_t)AT_LOW_OUTPUT);


    // Jika mencapai batas bawah
    if (temperatureC <= setpoint - AT_HYSTERESIS)
    {
      // Simpan trough
      if (troughCount < 8)
      {
        troughValues[troughCount] =
          temperatureC;

        troughCount++;
      }


      Serial.print("TROUGH #");
      Serial.print(troughCount);
      Serial.print(" = ");
      Serial.print(temperatureC, 2);
      Serial.println(" C");


      relayHigh = true;

      setHeaterPWM((uint8_t)AT_HIGH_OUTPUT);
    }
  }


  // ---------------------------------------------------
  // Cek apakah data cukup
  // ---------------------------------------------------

  if (
    peakCount >= AT_REQUIRED_PEAKS &&
    troughCount >= AT_REQUIRED_TROUGHS
  )
  {
    finishAutotune();
  }
}


// =====================================================
// PROSES PID NORMAL
// =====================================================

void processPID()
{
  // Pastikan PID aktif

  if (hotendPID.Compute())
  {
    pidOutput =
      constrain(pidOutput, 0, 255);

    setHeaterPWM((uint8_t)pidOutput);
  }
}


// =====================================================
// SAFETY
// =====================================================

bool checkSafety()
{
  // Sensor invalid

  if (isnan(temperatureC))
  {
    setHeaterPWM(0);

    Serial.println();
    Serial.println("ERROR: SENSOR NTC INVALID!");
    Serial.println("HEATER OFF.");

    mode = MODE_FAULT;

    return false;
  }


  // Overtemperature

  if (temperatureC >= MAX_TEMP)
  {
    setHeaterPWM(0);

    Serial.println();
    Serial.println("========================================");
    Serial.println("          OVER TEMPERATURE!");
    Serial.println("========================================");

    Serial.print("Temperature = ");
    Serial.print(temperatureC, 2);
    Serial.println(" C");

    Serial.println("HEATER OFF.");

    mode = MODE_FAULT;

    return false;
  }


  return true;
}


// =====================================================
// SERIAL COMMAND
// =====================================================

void processSerial()
{
  if (!Serial.available())
  {
    return;
  }


  String command =
    Serial.readStringUntil('\n');

  command.trim();


  // ---------------------------------------------------
  // AUTOTUNE
  // ---------------------------------------------------

  if (command.equalsIgnoreCase("a"))
  {
    if (
      mode != MODE_AUTOTUNE_WARMUP &&
      mode != MODE_AUTOTUNE_RELAY
    )
    {
      startAutotune();
    }

    return;
  }


  // ---------------------------------------------------
  // PID NORMAL
  // ---------------------------------------------------

  if (command.equalsIgnoreCase("p"))
  {
    if (mode != MODE_FAULT)
    {
      hotendPID.SetTunings(Kp, Ki, Kd);
      hotendPID.SetMode(AUTOMATIC);

      mode = MODE_PID;

      Serial.println();
      Serial.println("Mode PID normal aktif.");
    }

    return;
  }


  // ---------------------------------------------------
  // STOP HEATER
  // ---------------------------------------------------

  if (command.equalsIgnoreCase("o"))
  {
    setHeaterPWM(0);

    hotendPID.SetMode(MANUAL);

    mode = MODE_STOPPED;

    Serial.println();
    Serial.println("HEATER OFF.");
    Serial.println("Mode STOPPED.");

    return;
  }


  // ---------------------------------------------------
  // SETPOINT
  //
  // Contoh:
  // s260
  // s250
  // s270
  // ---------------------------------------------------

  if (
    command.length() > 1 &&
    command.charAt(0) == 's'
  )
  {
    double newSetpoint =
      command.substring(1).toFloat();


    if (
      newSetpoint >= 50 &&
      newSetpoint <= 270
    )
    {
      setpoint = newSetpoint;

      Serial.print("Setpoint = ");
      Serial.print(setpoint);
      Serial.println(" C");
    }
    else
    {
      Serial.println(
        "Setpoint harus 50-270 C."
      );
    }

    return;
  }


  // ---------------------------------------------------
  // TAMPILKAN PARAMETER
  // ---------------------------------------------------

  if (command.equalsIgnoreCase("k"))
  {
    Serial.println();
    Serial.println("===== PID PARAMETERS =====");

    Serial.print("Kp = ");
    Serial.println(Kp, 6);

    Serial.print("Ki = ");
    Serial.println(Ki, 6);

    Serial.print("Kd = ");
    Serial.println(Kd, 6);

    Serial.print("Setpoint = ");
    Serial.print(setpoint);
    Serial.println(" C");

    Serial.println();

    return;
  }


  // ---------------------------------------------------
  // HELP
  // ---------------------------------------------------

  if (command.equalsIgnoreCase("h"))
  {
    Serial.println();
    Serial.println("===== COMMAND =====");

    Serial.println("a     = mulai autotune");
    Serial.println("p     = PID normal");
    Serial.println("o     = heater OFF");
    Serial.println("k     = tampilkan Kp Ki Kd");
    Serial.println("s260  = setpoint 260 C");
    Serial.println("h     = bantuan");

    Serial.println();

    return;
  }


  Serial.println(
    "Command tidak dikenal. Ketik h untuk bantuan."
  );
}


// =====================================================
// STATUS SERIAL
// =====================================================

void printStatus()
{
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint < 1000)
  {
    return;
  }

  lastPrint = millis();


  Serial.print("TEMP=");
  Serial.print(temperatureC, 2);

  Serial.print(" C | SET=");
  Serial.print(setpoint, 1);

  Serial.print(" C | PWM=");
  Serial.print((int)pidOutput);

  Serial.print(" | MODE=");


  switch (mode)
  {
    case MODE_PID:
      Serial.println("PID");
      break;

    case MODE_AUTOTUNE_WARMUP:
      Serial.println("AUTOTUNE WARMUP");
      break;

    case MODE_AUTOTUNE_RELAY:
      Serial.println("AUTOTUNE RELAY");
      break;

    case MODE_STOPPED:
      Serial.println("STOPPED");
      break;

    case MODE_FAULT:
      Serial.println("FAULT");
      break;
  }
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
  // PWM
  // ---------------------------------------------------

  if (!ledcAttach(
        HEATER_PIN,
        PWM_FREQ,
        PWM_RES
      ))
  {
    Serial.println(
      "ERROR: ledcAttach gagal!"
    );

    while (1)
    {
      delay(1000);
    }
  }


  // Heater OFF
  setHeaterPWM(0);


  // ---------------------------------------------------
  // PID
  // ---------------------------------------------------

  hotendPID.SetOutputLimits(0, 255);

  hotendPID.SetSampleTime(500);

  hotendPID.SetMode(AUTOMATIC);


  // ---------------------------------------------------
  // INFO
  // ---------------------------------------------------

  Serial.println();
  Serial.println("========================================");
  Serial.println("        PET HOTEND CONTROLLER");
  Serial.println("========================================");

  Serial.println();

  Serial.print("Setpoint = ");
  Serial.print(setpoint);
  Serial.println(" C");

  Serial.print("Kp = ");
  Serial.println(Kp);

  Serial.print("Ki = ");
  Serial.println(Ki);

  Serial.print("Kd = ");
  Serial.println(Kd);

  Serial.println();

  Serial.println("Command:");
  Serial.println("a     -> Autotune");
  Serial.println("p     -> PID normal");
  Serial.println("o     -> Heater OFF");
  Serial.println("k     -> Tampilkan PID");
  Serial.println("s260  -> Setpoint 260 C");
  Serial.println("h     -> Help");

  Serial.println();
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  // ---------------------------------------------------
  // BACA TEMPERATUR
  // ---------------------------------------------------

  float newTemperature =
    readTemperature();


  if (!isnan(newTemperature))
  {
    temperatureC =
      newTemperature;
  }


  // ---------------------------------------------------
  // SAFETY
  // ---------------------------------------------------

  if (!checkSafety())
  {
    processSerial();

    delay(100);

    return;
  }


  // ---------------------------------------------------
  // SERIAL COMMAND
  // ---------------------------------------------------

  processSerial();


  // ---------------------------------------------------
  // MODE
  // ---------------------------------------------------

  switch (mode)
  {
    case MODE_PID:

      processPID();

      break;


    case MODE_AUTOTUNE_WARMUP:

      processAutotuneWarmup();

      break;


    case MODE_AUTOTUNE_RELAY:

      processRelayAutotune();

      break;


    case MODE_STOPPED:

      setHeaterPWM(0);

      break;


    case MODE_FAULT:

      setHeaterPWM(0);

      break;
  }


  // ---------------------------------------------------
  // STATUS
  // ---------------------------------------------------

  printStatus();


  delay(50);
}