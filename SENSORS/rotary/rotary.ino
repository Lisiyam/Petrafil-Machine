// =====================================================
// ROTARY ENCODER 1
// =====================================================
#define ENC1_CLK 32
#define ENC1_DT  33

// =====================================================
// ROTARY ENCODER 2
// =====================================================
#define ENC2_CLK 18
#define ENC2_DT  19
#define ENC2_SW  23

// =====================================================
// COUNTER
// =====================================================
int counter1 = 0;
int counter2 = 0;


// =====================================================
// STATE ENCODER
// =====================================================
uint8_t lastState1;
uint8_t lastState2;

int8_t position1 = 0;
int8_t position2 = 0;


// =====================================================
// QUADRATURE TRANSITION TABLE
// =====================================================
//
// State:
//
// 00
// 01
// 11
// 10
//
// Hanya transisi yang valid yang dihitung.
// Transisi akibat bounce / loncatan state akan diabaikan.
// =====================================================

const int8_t transitionTable[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};


// =====================================================
// SETUP
// =====================================================
void setup() {

    Serial.begin(115200);

    // -------------------------
    // Encoder 1
    // -------------------------
    pinMode(ENC1_CLK, INPUT_PULLUP);
    pinMode(ENC1_DT, INPUT_PULLUP);

    // -------------------------
    // Encoder 2
    // -------------------------
    pinMode(ENC2_CLK, INPUT_PULLUP);
    pinMode(ENC2_DT, INPUT_PULLUP);
    pinMode(ENC2_SW, INPUT_PULLUP);


    // -------------------------
    // Initial state
    // -------------------------
    lastState1 =
        (digitalRead(ENC1_CLK) << 1) |
         digitalRead(ENC1_DT);

    lastState2 =
        (digitalRead(ENC2_CLK) << 1) |
         digitalRead(ENC2_DT);


    Serial.println();
    Serial.println("=================================");
    Serial.println("     ROTARY ENCODER TEST");
    Serial.println("=================================");
    Serial.println("ENCODER 1 : CLK=32 DT=33");
    Serial.println("ENCODER 2 : CLK=26 DT=27 SW=14");
    Serial.println();
}


// =====================================================
// LOOP
// =====================================================
void loop() {

    // =================================================
    // ENCODER 1
    // =================================================

    uint8_t currentState1 =
        (digitalRead(ENC1_CLK) << 1) |
         digitalRead(ENC1_DT);

    if (currentState1 != lastState1) {

        uint8_t index =
            (lastState1 << 2) | currentState1;

        int8_t movement =
            transitionTable[index];

        position1 += movement;

        lastState1 = currentState1;


        // ------------------------------------------------
        // Satu detent biasanya = 4 valid transitions
        // ------------------------------------------------
        if (position1 >= 4) {

            counter1++;
            position1 = 0;

            Serial.print("[ENCODER 1] KANAN | Counter = ");
            Serial.println(counter1);
        }

        else if (position1 <= -4) {

            counter1--;
            position1 = 0;

            Serial.print("[ENCODER 1] KIRI  | Counter = ");
            Serial.println(counter1);
        }
    }


    // =================================================
    // ENCODER 2
    // =================================================

    uint8_t currentState2 =
        (digitalRead(ENC2_CLK) << 1) |
         digitalRead(ENC2_DT);

    if (currentState2 != lastState2) {

        uint8_t index =
            (lastState2 << 2) | currentState2;

        int8_t movement =
            transitionTable[index];

        position2 += movement;

        lastState2 = currentState2;


        // ------------------------------------------------
        // Satu detent
        // ------------------------------------------------
        if (position2 >= 4) {

            counter2++;
            position2 = 0;

            Serial.print("[ENCODER 2] KANAN | Counter = ");
            Serial.println(counter2);
        }

        else if (position2 <= -4) {

            counter2--;
            position2 = 0;

            Serial.print("[ENCODER 2] KIRI  | Counter = ");
            Serial.println(counter2);
        }
    }


    // =================================================
    // BUTTON ENCODER 2
    // =================================================

    static bool lastSW2 = HIGH;

    bool currentSW2 = digitalRead(ENC2_SW);

    if (lastSW2 == HIGH && currentSW2 == LOW) {

        Serial.println("[ENCODER 2] BUTTON PRESSED");

        delay(50);
    }

    lastSW2 = currentSW2;
}