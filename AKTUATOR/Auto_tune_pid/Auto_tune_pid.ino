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
// SPI
// =====================================================

SPISettings adsSPI(1000000, MSBFIRST, SPI_MODE1);


// =====================================================
// ADS1118 CONFIG
// =====================================================

const uint16_t ADS_CONFIG = 0xC383;


// =====================================================
// NTC
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

// Heater akan dimatikan jika mencapai suhu ini
const double MAX_TEMP = 280.0;


// =====================================================
// TEMPERATURE & PWM
// =====================================================

double temperatureC = 0.0;

// PWM AKTUAL heater
uint8_t heaterPWM = 0;


// =====================================================
// PID
// =====================================================

double pidOutput = 0.0;

// Nilai awal
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
// MODE
// =====================================================

enum SystemMode
{
  MODE_PID,
  MODE_AUTOTUNE_WARMUP,
  MODE_AUTOTUNE_RELAY,
  MODE_STOPPED,
  MODE_FAULT
};

SystemMode mode = MODE_STOPPED;


// =====================================================
// AUTOTUNE
// =====================================================

// Warmup menggunakan full power
const uint8_t AT_WARMUP_PWM = 255;

// Mulai relay ketika suhu sudah mendekati target
const double AT_START_TEMP = 258.0;


// Relay HIGH/LOW
const uint8_t AT_HIGH_PWM = 255;
const uint8_t AT_LOW_PWM  = 0;


// Hysteresis relay
const double AT_HYSTERESIS = 5.0;


// Jumlah osilasi yang dibutuhkan
const int AT_REQUIRED_PEAKS = 6;
const int AT_REQUIRED_TROUGHS = 6;


// Data peak/trough
double peakValues[8];
double troughValues[8];

unsigned long peakTimes[8];

int peakCount = 0;
int troughCount = 0;

bool relayHigh = true;


// =====================================================
// WARMUP MONITOR
// =====================================================

unsigned long warmupStartTime = 0;

double warmupStartTemperature = 0;

unsigned long lastWarmupCheck = 0;

double lastWarmupTemperature = 0;


// =====================================================
// SET HEATER PWM
// =====================================================

void setHeaterPWM(uint8_t pwm)
{
  heaterPWM = pwm;

  ledcWrite(
    HEATER_PIN,
    pwm
  );
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
// NTC -> TEMPERATURE
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
// READ TEMPERATURE
// =====================================================

float readTemperature()
{
  // Start conversion
  readADS1118();

  // ADS1118 @ 128 SPS
  delay(10);

  // Read conversion result
  uint16_t raw = readADS1118();

  int16_t signedRaw =
    (int16_t)raw;


  // ADC -> voltage

  float voltage =
    signedRaw * 4.096 / 32768.0;


  // Voltage -> NTC resistance

  float ntcResistance = NAN;


  if (
    voltage > 0.001 &&
    voltage < VCC
  )
  {
    ntcResistance =
      R_BOTTOM *
      ((VCC / voltage) - 1.0);
  }


  // Resistance -> temperature

  if (!isnan(ntcResistance))
  {
    return calculateTemperature(
      ntcResistance
    );
  }


  return NAN;
}


// =====================================================
// RESET AUTOTUNE
// =====================================================

void resetAutotuneData()
{
  peakCount = 0;
  troughCount = 0;

  for (int i = 0; i < 8; i++)
  {
    peakValues[i] = 0;
    troughValues[i] = 0;
    peakTimes[i] = 0;
  }
}


// =====================================================
// START AUTOTUNE
// =====================================================

void startAutotune()
{
  Serial.println();
  Serial.println("========================================");
  Serial.println("        AUTOTUNE DIMULAI");
  Serial.println("========================================");

  Serial.println();
  Serial.println("WARNING:");
  Serial.println("- Jangan memasukkan PET.");
  Serial.println("- Pastikan heater dan NTC terpasang.");
  Serial.println("- Pastikan sistem dapat membuang panas.");
  Serial.println();


  resetAutotuneData();


  warmupStartTime = millis();

  lastWarmupCheck = millis();

  warmupStartTemperature =
    temperatureC;

  lastWarmupTemperature =
    temperatureC;


  // PID tidak digunakan saat autotune
  hotendPID.SetMode(MANUAL);


  // ===================================================
  // FULL POWER
  // ===================================================

  setHeaterPWM(
    AT_WARMUP_PWM
  );


  mode =
    MODE_AUTOTUNE_WARMUP;


  Serial.println(
    "Warmup PWM = 255"
  );

  Serial.println(
    "Menunggu suhu mencapai 250 C..."
  );

  Serial.println();
}


// =====================================================
// START RELAY AUTOTUNE
// =====================================================

void startRelayAutotune()
{
  Serial.println();
  Serial.println("========================================");
  Serial.println("        RELAY AUTOTUNE DIMULAI");
  Serial.println("========================================");

  Serial.print(
    "Setpoint = "
  );

  Serial.print(
    setpoint
  );

  Serial.println(
    " C"
  );


  Serial.println(
    "Relay HIGH = 255"
  );

  Serial.println(
    "Relay LOW  = 0"
  );

  Serial.print(
    "Hysteresis = +/- "
  );

  Serial.print(
    AT_HYSTERESIS
  );

  Serial.println(
    " C"
  );

  Serial.println();


  resetAutotuneData();


  relayHigh = true;


  setHeaterPWM(
    AT_HIGH_PWM
  );


  mode =
    MODE_AUTOTUNE_RELAY;
}


// =====================================================
// WARMUP PROCESS
// =====================================================

void processAutotuneWarmup()
{
  // Selalu full power
  setHeaterPWM(
    AT_WARMUP_PWM
  );


  // ---------------------------------------------------
  // Sudah mencapai suhu untuk mulai autotune?
  // ---------------------------------------------------

  if (
    temperatureC >=
    AT_START_TEMP
  )
  {
    Serial.println();
    Serial.println(
      "Suhu sudah mencapai 250 C."
    );

    Serial.println(
      "Masuk ke RELAY AUTOTUNE."
    );

    startRelayAutotune();

    return;
  }


  // ---------------------------------------------------
  // MONITOR WARMUP
  // ---------------------------------------------------

  if (
    millis() - lastWarmupCheck >= 30000
  )
  {
    double deltaTemperature =
      temperatureC -
      lastWarmupTemperature;


    Serial.print(
      "Warmup 30s: "
    );

    Serial.print(
      deltaTemperature,
      2
    );

    Serial.println(
      " C"
    );


    // Jika kenaikan sangat kecil
    if (
      deltaTemperature < 0.5
    )
    {
      Serial.println(
        "WARNING: kenaikan suhu sangat kecil."
      );

      Serial.println(
        "Periksa heater, MOSFET, power supply,"
      );

      Serial.println(
        "thermal contact, dan insulation."
      );

      Serial.println();
    }


    lastWarmupTemperature =
      temperatureC;

    lastWarmupCheck =
      millis();
  }
}


// =====================================================
// RELAY AUTOTUNE
// =====================================================

void processRelayAutotune()
{
  unsigned long now =
    millis();


  // ---------------------------------------------------
  // SAFETY
  // ---------------------------------------------------

  if (
    temperatureC >= MAX_TEMP
  )
  {
    setHeaterPWM(0);

    Serial.println();
    Serial.println(
      "AUTOTUNE ABORT!"
    );

    Serial.println(
      "Overtemperature."
    );

    mode =
      MODE_FAULT;

    return;
  }


  // ---------------------------------------------------
  // HIGH
  // ---------------------------------------------------

  if (relayHigh)
  {
    setHeaterPWM(
      AT_HIGH_PWM
    );


    if (
      temperatureC >=
      setpoint + AT_HYSTERESIS
    )
    {
      // Simpan peak
      if (peakCount < 8)
      {
        peakValues[
          peakCount
        ] = temperatureC;

        peakTimes[
          peakCount
        ] = now;

        peakCount++;
      }


      Serial.print(
        "PEAK #"
      );

      Serial.print(
        peakCount
      );

      Serial.print(
        " = "
      );

      Serial.print(
        temperatureC,
        2
      );

      Serial.println(
        " C"
      );


      relayHigh = false;


      setHeaterPWM(
        AT_LOW_PWM
      );
    }
  }


  // ---------------------------------------------------
  // LOW
  // ---------------------------------------------------

  else
  {
    setHeaterPWM(
      AT_LOW_PWM
    );


    if (
      temperatureC <=
      setpoint - AT_HYSTERESIS
    )
    {
      // Simpan trough

      if (troughCount < 8)
      {
        troughValues[
          troughCount
        ] = temperatureC;

        troughCount++;
      }


      Serial.print(
        "TROUGH #"
      );

      Serial.print(
        troughCount
      );

      Serial.print(
        " = "
      );

      Serial.print(
        temperatureC,
        2
      );

      Serial.println(
        " C"
      );


      relayHigh = true;


      setHeaterPWM(
        AT_HIGH_PWM
      );
    }
  }


  // ---------------------------------------------------
  // CHECK DATA
  // ---------------------------------------------------

  if (
    peakCount >=
      AT_REQUIRED_PEAKS
    &&
    troughCount >=
      AT_REQUIRED_TROUGHS
  )
  {
    finishAutotune();
  }
}


// =====================================================
// FINISH AUTOTUNE
// =====================================================

void finishAutotune()
{
  setHeaterPWM(0);


  Serial.println();
  Serial.println("========================================");
  Serial.println("        AUTOTUNE SELESAI");
  Serial.println("========================================");


  // ---------------------------------------------------
  // Average peak
  // ---------------------------------------------------

  double peakAverage = 0;


  for (
    int i = 0;
    i < peakCount;
    i++
  )
  {
    peakAverage +=
      peakValues[i];
  }


  peakAverage /=
    peakCount;


  // ---------------------------------------------------
  // Average trough
  // ---------------------------------------------------

  double troughAverage = 0;


  for (
    int i = 0;
    i < troughCount;
    i++
  )
  {
    troughAverage +=
      troughValues[i];
  }


  troughAverage /=
    troughCount;


  // ---------------------------------------------------
  // Oscillation amplitude
  // ---------------------------------------------------

  double amplitude =
    (
      peakAverage -
      troughAverage
    ) / 2.0;


  // ---------------------------------------------------
  // Period
  // ---------------------------------------------------

  double periodSum = 0;

  int periodCount = 0;


  for (
    int i = 1;
    i < peakCount;
    i++
  )
  {
    double period =
      (
        peakTimes[i] -
        peakTimes[i - 1]
      ) / 1000.0;


    periodSum += period;

    periodCount++;
  }


  if (
    amplitude <= 0 ||
    periodCount <= 0
  )
  {
    Serial.println(
      "Autotuning gagal."
    );

    Serial.println(
      "Data osilasi tidak valid."
    );

    mode =
      MODE_STOPPED;

    return;
  }


  double Pu =
    periodSum /
    periodCount;


  // ---------------------------------------------------
  // Ultimate gain
  // ---------------------------------------------------

  double Ku =
    (
      4.0 *
      (
        (AT_HIGH_PWM - AT_LOW_PWM) /
        2.0
      )
    )
    /
    (
      PI *
      amplitude
    );


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
  // SIMPAN
  // ---------------------------------------------------

  Kp = newKp;
  Ki = newKi;
  Kd = newKd;


  hotendPID.SetTunings(
    Kp,
    Ki,
    Kd
  );


  // ---------------------------------------------------
  // HASIL
  // ---------------------------------------------------

  Serial.println();

  Serial.print(
    "Peak average   = "
  );

  Serial.print(
    peakAverage,
    3
  );

  Serial.println(
    " C"
  );


  Serial.print(
    "Trough average = "
  );

  Serial.print(
    troughAverage,
    3
  );

  Serial.println(
    " C"
  );


  Serial.print(
    "Amplitude      = "
  );

  Serial.print(
    amplitude,
    3
  );

  Serial.println(
    " C"
  );


  Serial.print(
    "Pu             = "
  );

  Serial.print(
    Pu,
    3
  );

  Serial.println(
    " s"
  );


  Serial.print(
    "Ku             = "
  );

  Serial.println(
    Ku,
    6
  );


  Serial.println();
  Serial.println(
    "PID HASIL AUTOTUNE:"
  );


  Serial.print(
    "Kp = "
  );

  Serial.println(
    Kp,
    6
  );


  Serial.print(
    "Ki = "
  );

  Serial.println(
    Ki,
    6
  );


  Serial.print(
    "Kd = "
  );

  Serial.println(
    Kd,
    6
  );


  Serial.println();


  // ---------------------------------------------------
  // Masuk PID
  // ---------------------------------------------------

  pidOutput = 0;


  hotendPID.SetMode(
    AUTOMATIC
  );


  mode =
    MODE_PID;


  Serial.println(
    "Mode PID normal aktif."
  );

  Serial.println();
}


// =====================================================
// PID NORMAL
// =====================================================

void processPID()
{
  if (
    hotendPID.Compute()
  )
  {
    pidOutput =
      constrain(
        pidOutput,
        0,
        255
      );


    setHeaterPWM(
      (uint8_t)pidOutput
    );
  }
}


// =====================================================
// SAFETY
// =====================================================

bool checkSafety()
{
  // Sensor invalid

  if (
    isnan(temperatureC)
  )
  {
    setHeaterPWM(0);

    Serial.println();
    Serial.println(
      "ERROR: NTC INVALID!"
    );

    Serial.println(
      "HEATER OFF."
    );

    mode =
      MODE_FAULT;

    return false;
  }


  // Overtemperature

  if (
    temperatureC >= MAX_TEMP
  )
  {
    setHeaterPWM(0);

    Serial.println();
    Serial.println(
      "========================================"
    );

    Serial.println(
      "          OVER TEMPERATURE!"
    );

    Serial.println(
      "========================================"
    );


    Serial.print(
      "Temperature = "
    );

    Serial.print(
      temperatureC,
      2
    );

    Serial.println(
      " C"
    );


    Serial.println(
      "HEATER OFF."
    );


    mode =
      MODE_FAULT;

    return false;
  }


  return true;
}


// =====================================================
// SERIAL COMMAND
// =====================================================

void processSerial()
{
  if (
    !Serial.available()
  )
  {
    return;
  }


  String command =
    Serial.readStringUntil(
      '\n'
    );


  command.trim();


  // -----------------------------------------------
  // AUTOTUNE
  // -----------------------------------------------

  if (
    command.equalsIgnoreCase("a")
  )
  {
    if (
      mode !=
        MODE_AUTOTUNE_WARMUP
      &&
      mode !=
        MODE_AUTOTUNE_RELAY
    )
    {
      startAutotune();
    }

    return;
  }


  // -----------------------------------------------
  // PID
  // -----------------------------------------------

  if (
    command.equalsIgnoreCase("p")
  )
  {
    if (
      mode != MODE_FAULT
    )
    {
      hotendPID.SetTunings(
        Kp,
        Ki,
        Kd
      );

      hotendPID.SetMode(
        AUTOMATIC
      );

      mode =
        MODE_PID;

      Serial.println();
      Serial.println(
        "PID normal aktif."
      );
    }

    return;
  }


  // -----------------------------------------------
  // OFF
  // -----------------------------------------------

  if (
    command.equalsIgnoreCase("o")
  )
  {
    setHeaterPWM(0);

    hotendPID.SetMode(
      MANUAL
    );

    mode =
      MODE_STOPPED;

    Serial.println();
    Serial.println(
      "HEATER OFF."
    );

    return;
  }


  // -----------------------------------------------
  // SETPOINT
  // s260
  // -----------------------------------------------

  if (
    command.length() > 1 &&
    command.charAt(0) == 's'
  )
  {
    double newSetpoint =
      command.substring(1)
      .toFloat();


    if (
      newSetpoint >= 50 &&
      newSetpoint <= 270
    )
    {
      setpoint =
        newSetpoint;


      Serial.print(
        "Setpoint = "
      );

      Serial.print(
        setpoint
      );

      Serial.println(
        " C"
      );
    }
    else
    {
      Serial.println(
        "Setpoint harus 50-270 C."
      );
    }

    return;
  }


  // -----------------------------------------------
  // PID PARAMETERS
  // -----------------------------------------------

  if (
    command.equalsIgnoreCase("k")
  )
  {
    Serial.println();
    Serial.println(
      "===== PID PARAMETERS ====="
    );


    Serial.print(
      "Kp = "
    );

    Serial.println(
      Kp,
      6
    );


    Serial.print(
      "Ki = "
    );

    Serial.println(
      Ki,
      6
    );


    Serial.print(
      "Kd = "
    );

    Serial.println(
      Kd,
      6
    );


    Serial.println();

    return;
  }


  // -----------------------------------------------
  // HELP
  // -----------------------------------------------

  if (
    command.equalsIgnoreCase("h")
  )
  {
    Serial.println();
    Serial.println(
      "===== COMMAND ====="
    );

    Serial.println(
      "a     = autotune"
    );

    Serial.println(
      "p     = PID normal"
    );

    Serial.println(
      "o     = heater OFF"
    );

    Serial.println(
      "k     = tampilkan PID"
    );

    Serial.println(
      "s260  = setpoint 260 C"
    );

    Serial.println(
      "h     = help"
    );

    Serial.println();

    return;
  }


  Serial.println(
    "Command tidak dikenal."
  );
}


// =====================================================
// STATUS
// =====================================================

void printStatus()
{
  static unsigned long lastPrint = 0;


  if (
    millis() - lastPrint <
    1000
  )
  {
    return;
  }


  lastPrint =
    millis();


  Serial.print(
    "TEMP="
  );

  Serial.print(
    temperatureC,
    2
  );


  Serial.print(
    " C | SET="
  );

  Serial.print(
    setpoint,
    1
  );


  Serial.print(
    " C | PWM="
  );

  // PWM AKTUAL
  Serial.print(
    heaterPWM
  );


  Serial.print(
    " | MODE="
  );


  switch (mode)
  {
    case MODE_PID:
      Serial.println(
        "PID"
      );
      break;


    case MODE_AUTOTUNE_WARMUP:
      Serial.println(
        "AUTOTUNE WARMUP"
      );
      break;


    case MODE_AUTOTUNE_RELAY:
      Serial.println(
        "AUTOTUNE RELAY"
      );
      break;


    case MODE_STOPPED:
      Serial.println(
        "STOPPED"
      );
      break;


    case MODE_FAULT:
      Serial.println(
        "FAULT"
      );
      break;
  }
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(
    115200
  );


  // ---------------------------------------------------
  // ADS1118
  // ---------------------------------------------------

  pinMode(
    ADS_CS,
    OUTPUT
  );

  digitalWrite(
    ADS_CS,
    HIGH
  );


  SPI.begin(
    ADS_SCK,
    ADS_MISO,
    ADS_MOSI,
    ADS_CS
  );


  // ---------------------------------------------------
  // PWM
  // ---------------------------------------------------

  if (
    !ledcAttach(
      HEATER_PIN,
      PWM_FREQ,
      PWM_RES
    )
  )
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

  hotendPID.SetOutputLimits(
    0,
    255
  );

  hotendPID.SetSampleTime(
    500
  );

  hotendPID.SetMode(
    MANUAL
  );


  // ---------------------------------------------------
  // INFO
  // ---------------------------------------------------

  Serial.println();
  Serial.println(
    "========================================"
  );

  Serial.println(
    "       PET HOTEND CONTROLLER"
  );

  Serial.println(
    "========================================"
  );

  Serial.println();

  Serial.println(
    "Commands:"
  );

  Serial.println(
    "a     -> Autotune"
  );

  Serial.println(
    "p     -> PID normal"
  );

  Serial.println(
    "o     -> Heater OFF"
  );

  Serial.println(
    "k     -> PID parameters"
  );

  Serial.println(
    "s260  -> Setpoint 260 C"
  );

  Serial.println(
    "h     -> Help"
  );

  Serial.println();
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  // ---------------------------------------------------
  // READ TEMPERATURE
  // ---------------------------------------------------

  float newTemperature =
    readTemperature();


  if (
    !isnan(newTemperature)
  )
  {
    temperatureC =
      newTemperature;
  }


  // ---------------------------------------------------
  // SAFETY
  // ---------------------------------------------------

  if (
    !checkSafety()
  )
  {
    processSerial();

    delay(100);

    return;
  }


  // ---------------------------------------------------
  // SERIAL
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