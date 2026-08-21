#include <Arduino.h>
#include <math.h>
#include <string.h>

// ============================================================
//                         PINS
// ============================================================
const uint8_t MOTOR_PIN_PWM = 9;
const uint8_t MOTOR_PIN_DIR = 8;
const uint8_t PIN_ENC_A     = 2;
const uint8_t PIN_ENC_B     = 3;

// Second driver for electromagnet
const uint8_t MAG_PIN_PWM   = 10;
const uint8_t MAG_PIN_DIR   = 7;

// ============================================================
//                    ENCODER / SCALING
// ============================================================
// HEDS x4 on motor/encoder side.
// Empirical basket-angle calibration from your measurements:
// basketDeg ≈ counts / 79.5
const long COUNTS_PER_REV = 2000;
const float COUNTS_PER_BASKET_DEG = 79.5f;
const float BASKET_ZERO_OFFSET_DEG = 0.0f;

// If motor direction is backwards, change ONE of these:
const bool MOTOR_DIR_INVERT = false;
const int ENCODER_DIR_SIGN  = -1;

// ============================================================
//                    SIMPLE POSITION CONTROL
// ============================================================
// P-only: pwm = Kp * position_error_deg
const float KP_PWM_PER_DEG = 1.5f; //############################################# 0.3-1.5-T=0.94；0.35-1.6-T=0.89；1.5-1.6-T=0.6;

// Max output PWM for motor command
// Raised because the dynamic minimum table can be multiplied by a factor.
const int PWM_MAX = 80;

// Dynamic minimum PWM LUT extracted from your gravity-feedforward test.
// Interpreted as step sections in basket angle magnitude.
const int DYN_MIN_LUT_SIZE = 10;
const float DYN_MIN_BREAK_DEG[DYN_MIN_LUT_SIZE] = {
  0.00f, 2.25f, 3.32f, 4.54f, 7.21f,
  8.69f, 10.28f, 11.69f, 13.55f, 14.70f
};
const int DYN_MIN_PWM[DYN_MIN_LUT_SIZE] = {
  13, 15, 17, 19, 25,
  29, 33, 41, 47, 51
};

// Scale factor applied to the LUT-derived minimum PWM.
const float DYN_MIN_FACTOR = 1.6f; //#################################################

// Global minimum PWM for downhill travel.
// Tune this value experimentally. Set to 0 to disable downhill minimum.
const int DOWNHILL_PWM_MIN = 15;

// Arrival condition in REAL basket degrees.
const float POSITION_TOL_DEG = 0.25f;

// Dwell at each end
const unsigned long DWELL_MS = 100;

// Safety timeout for one move
const unsigned long MOVE_TIMEOUT_MS = 5000;

// Targets in REAL BASKET DEGREES
const float TARGET_A_DEG = -15.0f;
const float TARGET_B_DEG =  15.0f;

// ============================================================
//                 MAGNET CURVE / LOOKUP SETTINGS
// ============================================================
const int CURVE_RESOLUTION = 680;
const char* CURVE_NAME = "custom1";
const float MAG_PWM_SCALE_DEFAULT = 55.0f;
const float MAG_PWM_SCALE_CUSTOM1 = 255.0f;
const bool MAG_ACTIVE_FORWARD = true;
const unsigned long MAGNET_TIMEOUT_MS = 1000;
const float CUSTOM1_PERIODS = 10.0f;

float curveTable[CURVE_RESOLUTION];
float curveGapDeg = 0.0f;

// ============================================================
//                      GLOBAL STATE
// ============================================================
volatile long g_counts = 0;

enum State {
  MOVING,
  DWELLING
};

State state = MOVING;

float currentTargetDeg = TARGET_A_DEG;
unsigned long moveStartMs  = 0;
unsigned long dwellStartMs = 0;

// Magnet process state
bool magProcessActive      = false;
bool magStartedThisMove    = false;
bool magTimedOutThisMove   = false;

int curveIndex = 0;
float lastMeasuredPosDeg = 0.0f;
int lastMagPwm = 0;
unsigned long lastCurveUpdateMs = 0;

float travelStartDeg = TARGET_A_DEG;
float travelEndDeg   = TARGET_B_DEG;

// ============================================================
//                    ENCODER ISR (x4)
// ============================================================
void isrEncA() {
  bool a = digitalReadFast(PIN_ENC_A);
  bool b = digitalReadFast(PIN_ENC_B);

  if (a == b) g_counts += ENCODER_DIR_SIGN;
  else        g_counts -= ENCODER_DIR_SIGN;
}

void isrEncB() {
  bool a = digitalReadFast(PIN_ENC_A);
  bool b = digitalReadFast(PIN_ENC_B);

  if (a != b) g_counts += ENCODER_DIR_SIGN;
  else        g_counts -= ENCODER_DIR_SIGN;
}

// ============================================================
//                         HELPERS
// ============================================================
long readCountsAtomic() {
  noInterrupts();
  long c = g_counts;
  interrupts();
  return c;
}

float countsToBasketDeg(long counts) {
  return (counts / COUNTS_PER_BASKET_DEG) + BASKET_ZERO_OFFSET_DEG;
}

int getDynamicPwmMinBase(float basketPosDeg) {
  float absDeg = fabsf(basketPosDeg);

  int pwmMin = DYN_MIN_PWM[0];
  for (int i = 0; i < DYN_MIN_LUT_SIZE; i++) {
    if (absDeg >= DYN_MIN_BREAK_DEG[i]) {
      pwmMin = DYN_MIN_PWM[i];
    } else {
      break;
    }
  }
  return pwmMin;
}

int getScaledDynamicPwmMin(float basketPosDeg) {
  int baseMin = getDynamicPwmMinBase(basketPosDeg);
  int scaledMin = (int)lroundf(baseMin * DYN_MIN_FACTOR);
  return constrain(scaledMin, 0, PWM_MAX);
}

float signf_simple(float x) {
  if (x > 0.0f) return 1.0f;
  if (x < 0.0f) return -1.0f;
  return 0.0f;
}

bool isUphillDesired(float basketPosDeg, float errDeg) {
  // Near 0 deg, either direction is climbing away from the bottom.
  if (fabsf(basketPosDeg) < POSITION_TOL_DEG) {
    return fabsf(errDeg) > POSITION_TOL_DEG;
  }

  // Desired motion is uphill if target direction points farther away from 0.
  // positive position + positive error => farther positive => uphill
  // negative position + negative error => farther negative => uphill
  return (basketPosDeg * errDeg) > 0.0f;
}

int getDownhillPwmMin() {
  return constrain(DOWNHILL_PWM_MIN, 0, PWM_MAX);
}

void setMotorEffort(int effort) {
  effort = constrain(effort, -PWM_MAX, PWM_MAX);

  if (MOTOR_DIR_INVERT) effort = -effort;

  if (effort >= 0) {
    digitalWriteFast(MOTOR_PIN_DIR, HIGH);
    analogWrite(MOTOR_PIN_PWM, effort);
  } else {
    digitalWriteFast(MOTOR_PIN_DIR, LOW);
    analogWrite(MOTOR_PIN_PWM, -effort);
  }
}

void stopMotor() {
  analogWrite(MOTOR_PIN_PWM, 0);
}

void setMagnetPwm(int pwm) {
  pwm = constrain(pwm, 0, 255);

  digitalWriteFast(MAG_PIN_DIR, HIGH);
  analogWrite(MAG_PIN_PWM, pwm);
}

float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

float getCurveScale(const char* functionName) {
  if (strcmp(functionName, "custom1") == 0) {
    return MAG_PWM_SCALE_CUSTOM1;
  }
  return MAG_PWM_SCALE_DEFAULT;
}

// ============================================================
//                 CURVE TABLE GENERATOR
// ============================================================
void generateCurveTable(float leftEdgeDeg,
                        float rightEdgeDeg,
                        const char* functionName,
                        int resolution,
                        float* outTable) {
  (void)leftEdgeDeg;
  (void)rightEdgeDeg;

  for (int i = 0; i < resolution; i++) {
    float u = (float)i / (float)(resolution - 1);
    float y = 0.0f;

    if (strcmp(functionName, "sin") == 0) {
      y = sinf(PI * u);
    }
    else if (strcmp(functionName, "cos") == 0) {
      y = 0.5f * (1.0f - cosf(2.0f * PI * u));
    }
    else if (strcmp(functionName, "sqw") == 0) {
      y = (u < 0.5f) ? 1.0f : 0.0f;
    }
    else if (strcmp(functionName, "custom1") == 0) {
      float phase = 2.0f * PI * CUSTOM1_PERIODS * u - 0.5f * PI;
      y = 0.5f * (1.0f + sinf(phase));
    }
    else if (strcmp(functionName, "custom2") == 0) {
      y = 1.0f - fabsf(2.0f * u - 1.0f);
    }
    else {
      y = 0.0f;
    }

    outTable[i] = clamp01(y);
  }
}

// ============================================================
//              MAGNET POSITION-INDEXED PLAYBACK
// ============================================================
bool isMovingTowardActiveDirection() {
  if (MAG_ACTIVE_FORWARD) {
    return fabsf(currentTargetDeg - TARGET_B_DEG) < 0.1f;
  } else {
    return fabsf(currentTargetDeg - TARGET_A_DEG) < 0.1f;
  }
}

void stopMagProcess() {
  magProcessActive = false;
  curveIndex = 0;
  lastMagPwm = 0;
  setMagnetPwm(0);
}

void startMagProcess(float currentPosDeg) {
  magProcessActive = true;
  curveIndex = 0;
  lastMeasuredPosDeg = currentPosDeg;
  lastCurveUpdateMs = millis();

  if (MAG_ACTIVE_FORWARD) {
    travelStartDeg = TARGET_A_DEG;
    travelEndDeg   = TARGET_B_DEG;
  } else {
    travelStartDeg = TARGET_B_DEG;
    travelEndDeg   = TARGET_A_DEG;
  }

  float scale = getCurveScale(CURVE_NAME);
  lastMagPwm = (int)lroundf(scale * curveTable[0]);
  setMagnetPwm(lastMagPwm);

  Serial.print("MAG PROCESS START curve=");
  Serial.println(CURVE_NAME);
}

void updateMagProcess(float currentPosDeg) {
  if (!magProcessActive) {
    setMagnetPwm(0);
    return;
  }

  if (fabsf(currentPosDeg - travelEndDeg) <= POSITION_TOL_DEG) {
    stopMagProcess();
    Serial.println("MAG PROCESS END: ARRIVAL");
    return;
  }

  float deltaAlongDeg = 0.0f;
  if (MAG_ACTIVE_FORWARD) {
    deltaAlongDeg = currentPosDeg - lastMeasuredPosDeg;
  } else {
    deltaAlongDeg = lastMeasuredPosDeg - currentPosDeg;
  }

  if (deltaAlongDeg <= 0.0f || deltaAlongDeg < curveGapDeg) {
    if (millis() - lastCurveUpdateMs >= MAGNET_TIMEOUT_MS) {
      stopMagProcess();
      magTimedOutThisMove = true;
      Serial.println("MAG PROCESS END: TIMEOUT");
    } else {
      setMagnetPwm(lastMagPwm);
    }
    return;
  }

  int stepsToAdvance = (int)floorf(deltaAlongDeg / curveGapDeg);
  if (stepsToAdvance < 1) {
    setMagnetPwm(lastMagPwm);
    return;
  }

  curveIndex += stepsToAdvance;
  if (curveIndex >= CURVE_RESOLUTION) {
    curveIndex = CURVE_RESOLUTION - 1;
  }

  if (MAG_ACTIVE_FORWARD) {
    lastMeasuredPosDeg += stepsToAdvance * curveGapDeg;
  } else {
    lastMeasuredPosDeg -= stepsToAdvance * curveGapDeg;
  }

  float scale = getCurveScale(CURVE_NAME);
  float y = curveTable[curveIndex];
  lastMagPwm = (int)lroundf(scale * y);
  setMagnetPwm(lastMagPwm);

  lastCurveUpdateMs = millis();
}

// ============================================================
//                    MOVE START / SETUP
// ============================================================
void startMoveTo(float targetDeg) {
  currentTargetDeg = targetDeg;
  moveStartMs = millis();
  state = MOVING;

  magStartedThisMove = false;
  magTimedOutThisMove = false;

  Serial.print("START target_deg=");
  Serial.println(currentTargetDeg);
}

void setup() {
  pinMode(MOTOR_PIN_PWM, OUTPUT);
  pinMode(MOTOR_PIN_DIR, OUTPUT);
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  pinMode(MAG_PIN_PWM, OUTPUT);
  pinMode(MAG_PIN_DIR, OUTPUT);

  stopMotor();
  digitalWriteFast(MOTOR_PIN_DIR, LOW);
  setMagnetPwm(0);
  digitalWriteFast(MAG_PIN_DIR, LOW);

  analogWriteFrequency(MOTOR_PIN_PWM, 20000);
  analogWriteFrequency(MAG_PIN_PWM, 20000);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrEncB, CHANGE);

  Serial.begin(115200);
  delay(300);

  noInterrupts();
  g_counts = 0;
  interrupts();

  generateCurveTable(TARGET_A_DEG, TARGET_B_DEG, CURVE_NAME, CURVE_RESOLUTION, curveTable);
  curveGapDeg = fabsf(TARGET_B_DEG - TARGET_A_DEG) / (float)(CURVE_RESOLUTION - 1);

  Serial.println("BOOT");
  Serial.println("Simple P basket-position swing: -15 <-> +15");
  Serial.println("Dynamic uphill-only minimum PWM enabled (applied before rounding)");
  Serial.println("Position-indexed magnet playback enabled");
  Serial.print("CURVE_NAME = ");
  Serial.println(CURVE_NAME);
  Serial.print("CURVE_RESOLUTION = ");
  Serial.println(CURVE_RESOLUTION);
  Serial.print("curveGapDeg = ");
  Serial.println(curveGapDeg, 4);
  Serial.print("DYN_MIN_FACTOR = ");
  Serial.println(DYN_MIN_FACTOR);
  Serial.print("MAG active direction = ");
  Serial.println(MAG_ACTIVE_FORWARD ? "A->B" : "B->A");
  Serial.print("MAG scale for this curve = ");
  Serial.println(getCurveScale(CURVE_NAME));

  startMoveTo(TARGET_A_DEG);
}

// ============================================================
//                           LOOP
// ============================================================
void loop() {
  long counts = readCountsAtomic();
  float posDeg = countsToBasketDeg(counts);
  float errDeg = currentTargetDeg - posDeg;

  if (state == MOVING) {
    if (fabsf(errDeg) <= POSITION_TOL_DEG) {
      stopMotor();
      stopMagProcess();

      Serial.print("ARRIVED target_deg=");
      Serial.print(currentTargetDeg);
      Serial.print("  pos_deg=");
      Serial.println(posDeg);

      dwellStartMs = millis();
      state = DWELLING;
    }
    else if (millis() - moveStartMs > MOVE_TIMEOUT_MS) {
      stopMotor();
      stopMagProcess();

      Serial.print("TIMEOUT target_deg=");
      Serial.print(currentTargetDeg);
      Serial.print("  pos_deg=");
      Serial.println(posDeg);

      while (1) { }
    }
    else {
      // Raw proportional command in float, before rounding.
      float pwmFloat = KP_PWM_PER_DEG * errDeg;

      // Apply minimum BEFORE rounding, based on desired motion direction.
      if (fabsf(errDeg) > POSITION_TOL_DEG) {
        float cmdSign = signf_simple(errDeg);
        float cmdMag = fabsf(pwmFloat);

        if (isUphillDesired(posDeg, errDeg)) {
          int pwmMinNow = getScaledDynamicPwmMin(posDeg);
          if (cmdMag < (float)pwmMinNow) {
            pwmFloat = cmdSign * (float)pwmMinNow;
          }
        } else {
          int pwmMinNow = getDownhillPwmMin();
          if (cmdMag < (float)pwmMinNow) {
            pwmFloat = cmdSign * (float)pwmMinNow;
          }
        }
      }

      int pwmCmd = (int)lroundf(pwmFloat);
      pwmCmd = constrain(pwmCmd, -PWM_MAX, PWM_MAX);
      setMotorEffort(pwmCmd);
    }
  }
  else if (state == DWELLING) {
    stopMotor();
    stopMagProcess();

    if (millis() - dwellStartMs >= DWELL_MS) {
      if (fabsf(currentTargetDeg - TARGET_A_DEG) < 0.1f) {
        startMoveTo(TARGET_B_DEG);
      } else {
        startMoveTo(TARGET_A_DEG);
      }
    }
  }

  bool activeMoveNow = (state == MOVING) && isMovingTowardActiveDirection();

  if (activeMoveNow) {
    if (!magStartedThisMove && !magTimedOutThisMove) {
      startMagProcess(posDeg);
      magStartedThisMove = true;
    }

    if (magProcessActive) {
      updateMagProcess(posDeg);
    } else {
      setMagnetPwm(0);
    }
  } else {
    stopMagProcess();
  }

  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs >= 100) {
    lastPrintMs = millis();

    float rawPwmFloat = KP_PWM_PER_DEG * errDeg;
    bool uphillNow = isUphillDesired(posDeg, errDeg);
    int pwmMinBase = getDynamicPwmMinBase(posDeg);
    int pwmMinScaled = getScaledDynamicPwmMin(posDeg);
    int pwmMinDownhill = getDownhillPwmMin();

    Serial.print("state=");
    Serial.print(state == MOVING ? "MOVING" : "DWELLING");
    Serial.print('\t');

    Serial.print("pos_deg=");
    Serial.print(posDeg);
    Serial.print('\t');

    Serial.print("target_deg=");
    Serial.print(currentTargetDeg);
    Serial.print('\t');

    Serial.print("err_deg=");
    Serial.print(errDeg);
    Serial.print('\t');

    Serial.print("raw_pwm=");
    Serial.print(rawPwmFloat);
    Serial.print('\t');

    Serial.print("uphill=");
    Serial.print(uphillNow ? "YES" : "NO");
    Serial.print('\t');

    Serial.print("pwmMinBase=");
    Serial.print(pwmMinBase);
    Serial.print('\t');

    Serial.print("pwmMinScaled=");
    Serial.print(pwmMinScaled);
    Serial.print('\t');

    Serial.print("mag_active=");
    Serial.print(magProcessActive ? "YES" : "NO");
    Serial.print('\t');

    Serial.print("mag_idx=");
    Serial.print(curveIndex);
    Serial.print('\t');

    Serial.print("mag_pwm=");
    Serial.println(lastMagPwm);
  }
}