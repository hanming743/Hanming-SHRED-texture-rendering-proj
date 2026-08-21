/*
  SHRED continuous-oscillation experiment firmware
  Version: shred-interactive-1.0

  Behavior:
    - Boot in IDLE with motor and magnet off.
    - Mac sends ZERO, ARM, START.
    - Basket continuously oscillates between leftDeg and rightDeg.
    - Mac may change KD at runtime to change the speed condition.
    - Mac may queue one waveform for one complete travel:
        QUEUE WAVEFORM <trial_id> <waveform> <L2R|R2L>
    - The waveform starts at the beginning of the next matching full
      travel and automatically stops at that travel's arrival.

  Controller:
    PWM = KP * position_error - KD * filtered_velocity
*/

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// Pins
const uint8_t MOTOR_PIN_PWM = 9;
const uint8_t MOTOR_PIN_DIR = 8;
const uint8_t PIN_ENC_A = 2;
const uint8_t PIN_ENC_B = 3;
const uint8_t MAG_PIN_PWM = 10;
const uint8_t MAG_PIN_DIR = 7;

// Calibration
const float COUNTS_PER_BASKET_DEG = 79.5f;
const float BASKET_ZERO_OFFSET_DEG = 0.0f;
const bool MOTOR_DIR_INVERT = false;
const int ENCODER_DIR_SIGN = +1;

const float CONFIG_LIMIT_LEFT_DEG = -16.0f;
const float CONFIG_LIMIT_RIGHT_DEG = 16.0f;
const float HARD_LIMIT_LEFT_DEG = -18.0f;
const float HARD_LIMIT_RIGHT_DEG = 18.0f;

// Dynamic uphill minimum LUT
const int DYN_MIN_LUT_SIZE = 10;
const float DYN_MIN_BREAK_DEG[DYN_MIN_LUT_SIZE] = {
  0.00f, 2.25f, 3.32f, 4.54f, 7.21f,
  8.69f, 10.28f, 11.69f, 13.55f, 14.70f
};
const int DYN_MIN_PWM[DYN_MIN_LUT_SIZE] = {
  13, 15, 17, 19, 25,
  29, 33, 41, 47, 51
};

// Curve storage
const int CURVE_RESOLUTION = 680;
const unsigned long MAGNET_TIMEOUT_MS = 1000UL;
float curveTable[CURVE_RESOLUTION];
float curveGapDeg = 0.0f;

struct RuntimeConfig {
  float leftDeg;
  float rightDeg;
  float kp;
  float kd;
  int pwmMax;
  float uphillFactor;
  int downhillMin;
  float positionTolDeg;
  unsigned long dwellMs;
  unsigned long moveTimeoutMs;
  float magScaleOverride;  // -1 = AUTO
  float custom1Periods;
  int streamHz;
};

RuntimeConfig config = {
  -15.0f,
   15.0f,
   1.5f,
   0.0f,
   80,
   1.6f,
   15,
   0.25f,
   100UL,
   5000UL,
   -1.0f,
   10.0f,
   50
};

enum DeviceState : uint8_t {
  STATE_IDLE = 0,
  STATE_ARMED,
  STATE_PREPOSITION,
  STATE_OSCILLATING,
  STATE_DWELLING,
  STATE_FAULT,
  STATE_ESTOP
};

DeviceState deviceState = STATE_IDLE;
char faultReason[48] = "NONE";

volatile long g_counts = 0;

float currentPosDeg = 0.0f;
float currentTargetDeg = 0.0f;
float previousVelocityPosDeg = 0.0f;
float filteredVelocityDegS = 0.0f;
unsigned long previousVelocityUs = 0UL;

unsigned long moveStartMs = 0UL;
unsigned long dwellStartMs = 0UL;
float nextTargetAfterDwell = 0.0f;
bool nextMoveIsFullTravel = false;

float lastControllerPwm = 0.0f;
int lastFinalPwm = 0;
int lastMinimumPwm = 0;
bool lastUphill = false;

const float STALL_FORWARD_PROGRESS_DEG = 0.05f;
const unsigned long STALL_TIMEOUT_MS = 3000UL;
float moveDirectionSign = 0.0f;
float stallAnchorDeg = 0.0f;
unsigned long stallWindowStartMs = 0UL;

// One-shot waveform queue
bool waveformPending = false;
bool waveformActive = false;

long pendingTrialId = -1;
long activeTrialId = -1;

char pendingWaveform[16] = "custom1";
char activeWaveform[16] = "custom1";

int pendingDirection = 0;  // 1=L2R, 2=R2L
int activeDirection = 0;

bool magProcessActive = false;
int curveIndex = 0;
float lastMeasuredPosDeg = 0.0f;
int lastMagPwm = 0;
unsigned long lastCurveUpdateMs = 0UL;
float travelStartDeg = -15.0f;
float travelEndDeg = 15.0f;

// Serial / telemetry / heartbeat
bool streamingEnabled = false;
unsigned long telemetryPeriodUs = 20000UL;
unsigned long lastTelemetryUs = 0UL;

const unsigned long HOST_TIMEOUT_MS = 2500UL;
unsigned long lastHostContactMs = 0UL;

const size_t RX_BUFFER_SIZE = 192;
char rxBuffer[RX_BUFFER_SIZE];
size_t rxLength = 0;

// Encoder ISR
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

// Helpers
long readCountsAtomic() {
  noInterrupts();
  long counts = g_counts;
  interrupts();
  return counts;
}

void zeroEncoderCounts() {
  noInterrupts();
  g_counts = 0;
  interrupts();

  currentPosDeg = BASKET_ZERO_OFFSET_DEG;
  currentTargetDeg = currentPosDeg;
  previousVelocityPosDeg = currentPosDeg;
  filteredVelocityDegS = 0.0f;
  previousVelocityUs = micros();
}

float countsToBasketDeg(long counts) {
  return (counts / COUNTS_PER_BASKET_DEG) + BASKET_ZERO_OFFSET_DEG;
}

float clamp01(float value) {
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

float signSimple(float value) {
  if (value > 0.0f) return 1.0f;
  if (value < 0.0f) return -1.0f;
  return 0.0f;
}

bool isMovingState() {
  return deviceState == STATE_PREPOSITION ||
         deviceState == STATE_OSCILLATING;
}

bool systemIsRunning() {
  return deviceState == STATE_PREPOSITION ||
         deviceState == STATE_OSCILLATING ||
         deviceState == STATE_DWELLING;
}

bool baseConfigurationEditable() {
  return deviceState == STATE_IDLE ||
         deviceState == STATE_ARMED;
}

const char* stateName() {
  switch (deviceState) {
    case STATE_IDLE: return "IDLE";
    case STATE_ARMED: return "ARMED";
    case STATE_PREPOSITION: return "PREPOSITION";
    case STATE_OSCILLATING: return "OSCILLATING";
    case STATE_DWELLING: return "DWELLING";
    case STATE_FAULT: return "FAULT";
    case STATE_ESTOP: return "ESTOP";
    default: return "UNKNOWN";
  }
}

const char* directionName(int directionCode) {
  if (directionCode == 1) return "L2R";
  if (directionCode == 2) return "R2L";
  return "NONE";
}

bool isSupportedWaveform(const char* name) {
  return strcmp(name, "sin") == 0 ||
         strcmp(name, "cos") == 0 ||
         strcmp(name, "sqw") == 0 ||
         strcmp(name, "custom1") == 0 ||
         strcmp(name, "custom2") == 0;
}

float getEffectiveMagScale(const char* waveformName) {
  if (config.magScaleOverride >= 0.0f) {
    return config.magScaleOverride;
  }

  if (strcmp(waveformName, "custom1") == 0) {
    return 255.0f;
  }

  return 55.0f;
}

// Outputs
void setMotorEffort(int effort) {
  effort = constrain(effort, -config.pwmMax, config.pwmMax);

  if (MOTOR_DIR_INVERT) {
    effort = -effort;
  }

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
  lastFinalPwm = 0;
}

void setMagnetPwm(int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWriteFast(MAG_PIN_DIR, HIGH);
  analogWrite(MAG_PIN_PWM, pwm);
}

void stopMagnet() {
  analogWrite(MAG_PIN_PWM, 0);
  lastMagPwm = 0;
}

void stopAllOutputs() {
  stopMotor();
  stopMagnet();
  magProcessActive = false;
  waveformActive = false;
}

// Dynamic minimum
int getDynamicPwmMinBase(float basketPosDeg) {
  float absDeg = fabsf(basketPosDeg);
  int pwmMin = DYN_MIN_PWM[0];

  for (int i = 0; i < DYN_MIN_LUT_SIZE; ++i) {
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
  int scaledMin = (int)lroundf(baseMin * config.uphillFactor);

  return constrain(scaledMin, 0, config.pwmMax);
}

int getDownhillPwmMin() {
  return constrain(config.downhillMin, 0, config.pwmMax);
}

bool isUphillDesired(float basketPosDeg, float errorDeg) {
  if (fabsf(basketPosDeg) < config.positionTolDeg) {
    return fabsf(errorDeg) > config.positionTolDeg;
  }

  return (basketPosDeg * errorDeg) > 0.0f;
}

// Waveform generation
void generateCurveTable(const char* waveformName) {
  for (int i = 0; i < CURVE_RESOLUTION; ++i) {
    float u = (float)i / (float)(CURVE_RESOLUTION - 1);
    float y = 0.0f;

    if (strcmp(waveformName, "sin") == 0) {
      y = sinf(PI * u);
    }
    else if (strcmp(waveformName, "cos") == 0) {
      y = 0.5f * (1.0f - cosf(2.0f * PI * u));
    }
    else if (strcmp(waveformName, "sqw") == 0) {
      y = (u < 0.5f) ? 1.0f : 0.0f;
    }
    else if (strcmp(waveformName, "custom1") == 0) {
      float phase =
          2.0f * PI * config.custom1Periods * u - 0.5f * PI;
      y = 0.5f * (1.0f + sinf(phase));
    }
    else if (strcmp(waveformName, "custom2") == 0) {
      y = 1.0f - fabsf(2.0f * u - 1.0f);
    }

    curveTable[i] = clamp01(y);
  }

  curveGapDeg =
      fabsf(config.rightDeg - config.leftDeg) /
      (float)(CURVE_RESOLUTION - 1);
}

// Events / faults
void sendSimpleEvent(const char* eventName) {
  Serial.print("EVENT,");
  Serial.print(micros());
  Serial.print(",");
  Serial.print(eventName);
  Serial.print(",state=");
  Serial.print(stateName());
  Serial.print(",basketDeg=");
  Serial.println(currentPosDeg, 4);
}

void sendWaveformEvent(const char* eventName) {
  Serial.print("EVENT,");
  Serial.print(micros());
  Serial.print(",");
  Serial.print(eventName);
  Serial.print(",trial=");
  Serial.print(activeTrialId);
  Serial.print(",waveform=");
  Serial.print(activeWaveform);
  Serial.print(",direction=");
  Serial.print(directionName(activeDirection));
  Serial.print(",state=");
  Serial.print(stateName());
  Serial.print(",basketDeg=");
  Serial.println(currentPosDeg, 4);
}

void enterFault(const char* reason) {
  stopAllOutputs();
  waveformPending = false;

  strncpy(faultReason, reason, sizeof(faultReason) - 1);
  faultReason[sizeof(faultReason) - 1] = '\0';

  deviceState = STATE_FAULT;

  Serial.print("FAULT,");
  Serial.print(micros());
  Serial.print(",reason=");
  Serial.print(faultReason);
  Serial.print(",basketDeg=");
  Serial.println(currentPosDeg, 4);
}

void enterEstop() {
  stopAllOutputs();
  waveformPending = false;

  strncpy(faultReason, "ESTOP", sizeof(faultReason) - 1);
  faultReason[sizeof(faultReason) - 1] = '\0';

  deviceState = STATE_ESTOP;
  sendSimpleEvent("ESTOP_LATCHED");
}

// Magnet playback
void stopMagProcess() {
  magProcessActive = false;
  curveIndex = 0;
  stopMagnet();
}

void finishActiveWaveform(const char* eventName) {
  bool wasActive = waveformActive;

  stopMagProcess();

  if (wasActive) {
    sendWaveformEvent(eventName);
  }

  waveformActive = false;
  activeTrialId = -1;
  activeDirection = 0;
}

void startQueuedWaveform() {
  strncpy(
      activeWaveform,
      pendingWaveform,
      sizeof(activeWaveform) - 1
  );
  activeWaveform[sizeof(activeWaveform) - 1] = '\0';

  activeTrialId = pendingTrialId;
  activeDirection = pendingDirection;

  waveformPending = false;
  waveformActive = true;
  magProcessActive = true;

  generateCurveTable(activeWaveform);

  curveIndex = 0;
  lastMeasuredPosDeg = currentPosDeg;
  lastCurveUpdateMs = millis();

  if (activeDirection == 1) {
    travelStartDeg = config.leftDeg;
    travelEndDeg = config.rightDeg;
  } else {
    travelStartDeg = config.rightDeg;
    travelEndDeg = config.leftDeg;
  }

  lastMagPwm = (int)lroundf(
      getEffectiveMagScale(activeWaveform) * curveTable[0]
  );

  setMagnetPwm(lastMagPwm);
  sendWaveformEvent("WAVEFORM_STARTED");
}

void updateMagProcess() {
  if (!waveformActive || !magProcessActive) {
    stopMagnet();
    return;
  }

  if (fabsf(currentPosDeg - travelEndDeg) <= config.positionTolDeg) {
    finishActiveWaveform("WAVEFORM_ENDED");
    return;
  }

  bool increasingTravel = travelEndDeg > travelStartDeg;

  float deltaAlongDeg =
      increasingTravel
      ? currentPosDeg - lastMeasuredPosDeg
      : lastMeasuredPosDeg - currentPosDeg;

  if (deltaAlongDeg <= 0.0f || deltaAlongDeg < curveGapDeg) {
    if (millis() - lastCurveUpdateMs >= MAGNET_TIMEOUT_MS) {
      finishActiveWaveform("WAVEFORM_TIMEOUT");
    } else {
      setMagnetPwm(lastMagPwm);
    }
    return;
  }

  int stepsToAdvance =
      (int)floorf(deltaAlongDeg / curveGapDeg);

  if (stepsToAdvance < 1) {
    setMagnetPwm(lastMagPwm);
    return;
  }

  curveIndex += stepsToAdvance;

  if (curveIndex >= CURVE_RESOLUTION) {
    curveIndex = CURVE_RESOLUTION - 1;
  }

  if (increasingTravel) {
    lastMeasuredPosDeg += stepsToAdvance * curveGapDeg;
  } else {
    lastMeasuredPosDeg -= stepsToAdvance * curveGapDeg;
  }

  lastMagPwm = (int)lroundf(
      getEffectiveMagScale(activeWaveform) * curveTable[curveIndex]
  );

  setMagnetPwm(lastMagPwm);
  lastCurveUpdateMs = millis();
}

// Kinematics
void updateKinematics() {
  currentPosDeg = countsToBasketDeg(readCountsAtomic());

  unsigned long nowUs = micros();

  if (previousVelocityUs == 0UL) {
    previousVelocityUs = nowUs;
    previousVelocityPosDeg = currentPosDeg;
    filteredVelocityDegS = 0.0f;
    return;
  }

  unsigned long elapsedUs = nowUs - previousVelocityUs;

  if (elapsedUs < 2000UL) {
    return;
  }

  float dt = elapsedUs * 1.0e-6f;
  float rawVelocity =
      (currentPosDeg - previousVelocityPosDeg) / dt;

  const float alpha = 0.20f;

  filteredVelocityDegS =
      alpha * rawVelocity +
      (1.0f - alpha) * filteredVelocityDegS;

  previousVelocityPosDeg = currentPosDeg;
  previousVelocityUs = nowUs;
}

// Oscillation state machine
void resetMoveSafetyWindows() {
  moveStartMs = millis();
  moveDirectionSign =
      signSimple(currentTargetDeg - currentPosDeg);

  stallAnchorDeg = currentPosDeg;
  stallWindowStartMs = millis();
}

bool currentTravelMatchesPendingDirection() {
  if (!waveformPending) {
    return false;
  }

  bool movingL2R =
      fabsf(currentTargetDeg - config.rightDeg) < 0.1f;

  bool movingR2L =
      fabsf(currentTargetDeg - config.leftDeg) < 0.1f;

  return
      (pendingDirection == 1 && movingL2R) ||
      (pendingDirection == 2 && movingR2L);
}

void beginMoveTo(
    float targetDeg,
    uint8_t stateCode,
    bool fullBoundaryTravel
) {
  currentTargetDeg = targetDeg;
  deviceState = (DeviceState)stateCode;

  resetMoveSafetyWindows();

  lastControllerPwm = 0.0f;
  lastFinalPwm = 0;
  lastMinimumPwm = 0;
  lastUphill = false;

  sendSimpleEvent(
      fullBoundaryTravel
      ? "OSCILLATION_TRAVEL_STARTED"
      : "PREPOSITION_STARTED"
  );

  if (
      fullBoundaryTravel &&
      currentTravelMatchesPendingDirection()
  ) {
    startQueuedWaveform();
  }
}

void beginDwell(
    float nextTargetDeg,
    bool nextFullTravel
) {
  stopMotor();

  if (waveformActive) {
    finishActiveWaveform("WAVEFORM_ENDED");
  } else {
    stopMagProcess();
  }

  nextTargetAfterDwell = nextTargetDeg;
  nextMoveIsFullTravel = nextFullTravel;

  deviceState = STATE_DWELLING;
  dwellStartMs = millis();

  sendSimpleEvent("DWELL_STARTED");
}

void startContinuousOscillation() {
  waveformPending = false;
  waveformActive = false;

  // First move only positions the basket at the left boundary.
  beginMoveTo(
      config.leftDeg,
      STATE_PREPOSITION,
      false
  );
}

void handleArrival() {
  stopMotor();

  if (deviceState == STATE_PREPOSITION) {
    beginDwell(
        config.rightDeg,
        true
    );
    return;
  }

  if (deviceState == STATE_OSCILLATING) {
    bool arrivedAtRight =
        fabsf(currentTargetDeg - config.rightDeg) < 0.1f;

    float nextTarget =
        arrivedAtRight
        ? config.leftDeg
        : config.rightDeg;

    beginDwell(
        nextTarget,
        true
    );
  }
}

void updateDwellState() {
  if (deviceState != STATE_DWELLING) {
    return;
  }

  stopMotor();
  stopMagnet();

  if (millis() - dwellStartMs < config.dwellMs) {
    return;
  }

  beginMoveTo(
      nextTargetAfterDwell,
      STATE_OSCILLATING,
      nextMoveIsFullTravel
  );
}

// Motor control / safety
void updateMotorControl() {
  if (!isMovingState()) {
    return;
  }

  if (
      currentPosDeg < HARD_LIMIT_LEFT_DEG ||
      currentPosDeg > HARD_LIMIT_RIGHT_DEG
  ) {
    enterFault("HARD_ANGLE_LIMIT");
    return;
  }

  if (millis() - moveStartMs > config.moveTimeoutMs) {
    enterFault("MOVE_TIMEOUT");
    return;
  }

  float errorDeg = currentTargetDeg - currentPosDeg;

  if (fabsf(errorDeg) <= config.positionTolDeg) {
    handleArrival();
    return;
  }

  float controllerPwm =
      config.kp * errorDeg -
      config.kd * filteredVelocityDegS;

  float pwmFloat = controllerPwm;

  bool commandPointsTowardTarget =
      (pwmFloat * errorDeg) >= 0.0f;

  bool uphillNow =
      isUphillDesired(currentPosDeg, errorDeg);

  int minimumPwm = 0;

  // Do not override derivative braking with a minimum floor.
  if (commandPointsTowardTarget) {
    float commandSign = signSimple(errorDeg);
    float commandMagnitude = fabsf(pwmFloat);

    if (uphillNow) {
      minimumPwm =
          getScaledDynamicPwmMin(currentPosDeg);
    } else {
      minimumPwm = getDownhillPwmMin();
    }

    if (commandMagnitude < (float)minimumPwm) {
      pwmFloat = commandSign * (float)minimumPwm;
    }
  }

  int pwmCommand = (int)lroundf(pwmFloat);

  pwmCommand = constrain(
      pwmCommand,
      -config.pwmMax,
      config.pwmMax
  );

  lastControllerPwm = controllerPwm;
  lastFinalPwm = pwmCommand;
  lastMinimumPwm = minimumPwm;
  lastUphill = uphillNow;

  setMotorEffort(pwmCommand);

  float forwardProgress =
      moveDirectionSign *
      (currentPosDeg - stallAnchorDeg);

  if (forwardProgress >= STALL_FORWARD_PROGRESS_DEG) {
    stallAnchorDeg = currentPosDeg;
    stallWindowStartMs = millis();
  }
  else if (
      abs(pwmCommand) > 0 &&
      millis() - stallWindowStartMs >= STALL_TIMEOUT_MS
  ) {
    enterFault("MOTOR_STALL");
    return;
  }

  if (
      deviceState == STATE_OSCILLATING &&
      waveformActive
  ) {
    updateMagProcess();
  } else if (!waveformActive) {
    stopMagProcess();
  }
}

// Telemetry / status
void sendTelemetryWhenDue() {
  if (!streamingEnabled) {
    return;
  }

  unsigned long nowUs = micros();

  if (nowUs - lastTelemetryUs < telemetryPeriodUs) {
    return;
  }

  lastTelemetryUs = nowUs;

  float errorDeg = currentTargetDeg - currentPosDeg;

  Serial.print("DATA,");
  Serial.print(nowUs);
  Serial.print(",");
  Serial.print(stateName());
  Serial.print(",");
  Serial.print(currentPosDeg, 4);
  Serial.print(",");
  Serial.print(currentTargetDeg, 4);
  Serial.print(",");
  Serial.print(errorDeg, 4);
  Serial.print(",");
  Serial.print(filteredVelocityDegS, 4);
  Serial.print(",");
  Serial.print(config.kd, 4);
  Serial.print(",");
  Serial.print(lastControllerPwm, 3);
  Serial.print(",");
  Serial.print(lastFinalPwm);
  Serial.print(",");
  Serial.print(lastMinimumPwm);
  Serial.print(",");
  Serial.print(lastUphill ? "YES" : "NO");
  Serial.print(",");
  Serial.print(lastMagPwm);
  Serial.print(",");
  Serial.print(curveIndex);
  Serial.print(",");
  Serial.println(waveformActive ? "YES" : "NO");
}

void printStatus() {
  Serial.print("STATUS state=");
  Serial.print(stateName());
  Serial.print(" basketDeg=");
  Serial.print(currentPosDeg, 4);
  Serial.print(" targetDeg=");
  Serial.print(currentTargetDeg, 4);
  Serial.print(" velocityDegS=");
  Serial.print(filteredVelocityDegS, 4);
  Serial.print(" kd=");
  Serial.print(config.kd, 4);
  Serial.print(" motorPwm=");
  Serial.print(lastFinalPwm);
  Serial.print(" magnetPwm=");
  Serial.print(lastMagPwm);
  Serial.print(" pending=");
  Serial.print(waveformPending ? "YES" : "NO");
  Serial.print(" active=");
  Serial.print(waveformActive ? "YES" : "NO");
  Serial.print(" fault=");
  Serial.println(faultReason);
}

void printConfig() {
  Serial.print("CONFIG leftDeg=");
  Serial.print(config.leftDeg, 3);
  Serial.print(" rightDeg=");
  Serial.print(config.rightDeg, 3);
  Serial.print(" kp=");
  Serial.print(config.kp, 4);
  Serial.print(" kd=");
  Serial.print(config.kd, 4);
  Serial.print(" pwmMax=");
  Serial.print(config.pwmMax);
  Serial.print(" uphillFactor=");
  Serial.print(config.uphillFactor, 3);
  Serial.print(" downhillMin=");
  Serial.print(config.downhillMin);
  Serial.print(" positionTolDeg=");
  Serial.print(config.positionTolDeg, 3);
  Serial.print(" dwellMs=");
  Serial.print(config.dwellMs);
  Serial.print(" moveTimeoutMs=");
  Serial.print(config.moveTimeoutMs);
  Serial.print(" magScaleSetting=");

  if (config.magScaleOverride < 0.0f) {
    Serial.print("AUTO");
  } else {
    Serial.print(config.magScaleOverride, 2);
  }

  Serial.print(" custom1Periods=");
  Serial.print(config.custom1Periods, 2);
  Serial.print(" streamHz=");
  Serial.println(config.streamHz);
}

// Parsing helpers
bool parseFloatStrict(const char* text, float& result) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char* endPointer = nullptr;
  float value = strtof(text, &endPointer);

  if (
      endPointer == text ||
      *endPointer != '\0' ||
      !isfinite(value)
  ) {
    return false;
  }

  result = value;
  return true;
}

bool parseLongStrict(const char* text, long& result) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char* endPointer = nullptr;
  long value = strtol(text, &endPointer, 10);

  if (
      endPointer == text ||
      *endPointer != '\0'
  ) {
    return false;
  }

  result = value;
  return true;
}

void acknowledgeSet(const char* key, const char* value) {
  Serial.print("ACK SET ");
  Serial.print(key);
  Serial.print(" value=");
  Serial.println(value);
}

void rejectSet(const char* key, const char* reason) {
  Serial.print("NACK SET ");
  Serial.print(key);
  Serial.print(" reason=");
  Serial.println(reason);
}

// SET commands
void handleSetCommand(char* command) {
  char* savePointer = nullptr;

  char* token = strtok_r(command, " ", &savePointer); // SET
  token = strtok_r(nullptr, " ", &savePointer);       // KEY

  if (token == nullptr) {
    Serial.println("NACK SET reason=missing_key");
    return;
  }

  char key[32];

  strncpy(key, token, sizeof(key) - 1);
  key[sizeof(key) - 1] = '\0';

  token = strtok_r(nullptr, " ", &savePointer);       // VALUE

  if (token == nullptr) {
    rejectSet(key, "missing_value");
    return;
  }

  if (strtok_r(nullptr, " ", &savePointer) != nullptr) {
    rejectSet(key, "too_many_arguments");
    return;
  }

  char valueText[32];

  strncpy(valueText, token, sizeof(valueText) - 1);
  valueText[sizeof(valueText) - 1] = '\0';

  bool runtimeKdChange = strcmp(key, "KD") == 0;
  bool runtimeStreamChange = strcmp(key, "STREAM_HZ") == 0;

  if (
      !baseConfigurationEditable() &&
      !runtimeKdChange &&
      !runtimeStreamChange
  ) {
    rejectSet(key, "device_busy");
    return;
  }

  float floatValue = 0.0f;
  long longValue = 0;

  if (strcmp(key, "LEFT") == 0) {
    if (!parseFloatStrict(valueText, floatValue)) {
      rejectSet(key, "invalid_number");
      return;
    }

    if (
        floatValue < CONFIG_LIMIT_LEFT_DEG ||
        floatValue >= config.rightDeg
    ) {
      rejectSet(
          key,
          "out_of_range_or_not_less_than_right"
      );
      return;
    }

    config.leftDeg = floatValue;
    acknowledgeSet(key, valueText);
    return;
  }

  if (strcmp(key, "RIGHT") == 0) {
    if (!parseFloatStrict(valueText, floatValue)) {
      rejectSet(key, "invalid_number");
      return;
    }

    if (
        floatValue > CONFIG_LIMIT_RIGHT_DEG ||
        floatValue <= config.leftDeg
    ) {
      rejectSet(
          key,
          "out_of_range_or_not_greater_than_left"
      );
      return;
    }

    config.rightDeg = floatValue;
    acknowledgeSet(key, valueText);
    return;
  }

  if (strcmp(key, "KP") == 0) {
    if (
        !parseFloatStrict(valueText, floatValue) ||
        floatValue < 0.0f ||
        floatValue > 20.0f
    ) {
      rejectSet(key, "out_of_range");
      return;
    }

    config.kp = floatValue;
    acknowledgeSet(key, valueText);
    return;
  }

  if (strcmp(key, "KD") == 0) {
    if (
        !parseFloatStrict(valueText, floatValue) ||
        floatValue < 0.0f ||
        floatValue > 10.0f
    ) {
      rejectSet(key, "out_of_range");
      return;
    }

    config.kd = floatValue;
    acknowledgeSet(key, valueText);
    return;
  }

  if (strcmp(key, "PWM_MAX") == 0) {
    if (
        !parseLongStrict(valueText, longValue) ||
        longValue < 0 ||
        longValue > 255
    ) {
      rejectSet(key, "out_of_range");
      return;
    }

    config.pwmMax = (int)longValue;
    acknowledgeSet(key, valueText);
    return;
  }

  if (strcmp(key, "UPHILL_FACTOR") == 0) {
    if (
        !parseFloatStrict(valueText, floatValue) ||
        floatValue < 0.0f ||
        floatValue > 5.0f
    ) {
      rejectSet(key, "out_of_range");
      return;
    }

    config.uphillFactor = floatValue;
    acknowledgeSet(key, valueText);
    return;
  }

  if (strcmp(key, "DOWNHILL_MIN") == 0) {
    if (
        !parseLongStrict(valueText, longValue) ||
        longValue < 0 ||
        longValue > 255
    ) {
      rejectSet(key, "out_of_range");
      return;
    }

    config.downhillMin = (int)longValue;
    acknowledgeSet(key, valueText);
    return;
  }

  if (strcmp(key, "POSITION_TOL") == 0) {
    if (
        !parseFloatStrict(valueText, floatValue) ||
        floatValue <= 0.0f ||
        floatValue > 2.0f
    ) {
      rejectSet(key, "out_of_range");
      return;
    }

    config.positionTolDeg = floatValue;
    acknowledgeSet(key, valueText);
    return;
  }

  if (strcmp(key, "DWELL_MS") == 0) {
    if (
        !parseLongStrict(valueText, longValue) ||
        longValue < 0 ||
        longValue > 60000
    ) {
      rejectSet(key, "out_of_range");
      return;
    }

    config.dwellMs = (unsigned long)longValue;
    acknowledgeSet(key, valueText);
    return;
  }

  if (strcmp(key, "MOVE_TIMEOUT_MS") == 0) {
    if (
        !parseLongStrict(valueText, longValue) ||
        longValue < 500 ||
        longValue > 120000
    ) {
      rejectSet(key, "out_of_range");
      return;
    }

    config.moveTimeoutMs = (unsigned long)longValue;
    acknowledgeSet(key, valueText);
    return;
  }

  if (strcmp(key, "MAG_SCALE") == 0) {
    if (strcmp(valueText, "AUTO") == 0) {
      config.magScaleOverride = -1.0f;
      acknowledgeSet(key, valueText);
      return;
    }

    if (
        !parseFloatStrict(valueText, floatValue) ||
        floatValue < 0.0f ||
        floatValue > 255.0f
    ) {
      rejectSet(key, "out_of_range");
      return;
    }

    config.magScaleOverride = floatValue;
    acknowledgeSet(key, valueText);
    return;
  }

  if (strcmp(key, "CUSTOM1_PERIODS") == 0) {
    if (
        !parseFloatStrict(valueText, floatValue) ||
        floatValue <= 0.0f ||
        floatValue > 100.0f
    ) {
      rejectSet(key, "out_of_range");
      return;
    }

    config.custom1Periods = floatValue;
    acknowledgeSet(key, valueText);
    return;
  }

  if (strcmp(key, "STREAM_HZ") == 0) {
    if (
        !parseLongStrict(valueText, longValue) ||
        longValue < 1 ||
        longValue > 200
    ) {
      rejectSet(key, "out_of_range");
      return;
    }

    config.streamHz = (int)longValue;
    telemetryPeriodUs =
        1000000UL /
        (unsigned long)config.streamHz;

    acknowledgeSet(key, valueText);
    return;
  }

  rejectSet(key, "unknown_key");
}

// Queue one-shot waveform
void handleQueueWaveformCommand(char* command) {
  if (!systemIsRunning()) {
    Serial.println(
        "NACK QUEUE WAVEFORM reason=system_not_oscillating"
    );
    return;
  }

  if (waveformPending || waveformActive) {
    Serial.println(
        "NACK QUEUE WAVEFORM reason=waveform_already_pending_or_active"
    );
    return;
  }

  char* savePointer = nullptr;

  char* token = strtok_r(command, " ", &savePointer); // QUEUE
  token = strtok_r(nullptr, " ", &savePointer);       // WAVEFORM
  token = strtok_r(nullptr, " ", &savePointer);       // trial id

  if (token == nullptr) {
    Serial.println(
        "NACK QUEUE WAVEFORM reason=missing_trial_id"
    );
    return;
  }

  long trialId = -1;

  if (
      !parseLongStrict(token, trialId) ||
      trialId < 0
  ) {
    Serial.println(
        "NACK QUEUE WAVEFORM reason=invalid_trial_id"
    );
    return;
  }

  token = strtok_r(nullptr, " ", &savePointer);       // waveform

  if (
      token == nullptr ||
      !isSupportedWaveform(token)
  ) {
    Serial.println(
        "NACK QUEUE WAVEFORM reason=unsupported_waveform"
    );
    return;
  }

  char waveformName[16];

  strncpy(
      waveformName,
      token,
      sizeof(waveformName) - 1
  );
  waveformName[sizeof(waveformName) - 1] = '\0';

  token = strtok_r(nullptr, " ", &savePointer);       // direction

  if (token == nullptr) {
    Serial.println(
        "NACK QUEUE WAVEFORM reason=missing_direction"
    );
    return;
  }

  int directionCode = 0;

  if (strcmp(token, "L2R") == 0) {
    directionCode = 1;
  }
  else if (strcmp(token, "R2L") == 0) {
    directionCode = 2;
  }
  else {
    Serial.println(
        "NACK QUEUE WAVEFORM reason=use_L2R_or_R2L"
    );
    return;
  }

  if (strtok_r(nullptr, " ", &savePointer) != nullptr) {
    Serial.println(
        "NACK QUEUE WAVEFORM reason=too_many_arguments"
    );
    return;
  }

  pendingTrialId = trialId;

  strncpy(
      pendingWaveform,
      waveformName,
      sizeof(pendingWaveform) - 1
  );
  pendingWaveform[sizeof(pendingWaveform) - 1] = '\0';

  pendingDirection = directionCode;
  waveformPending = true;

  Serial.print("ACK QUEUE WAVEFORM trial=");
  Serial.print(pendingTrialId);
  Serial.print(" waveform=");
  Serial.print(pendingWaveform);
  Serial.print(" direction=");
  Serial.println(directionName(pendingDirection));
}

// General commands
void trimCommand(char*& command) {
  while (*command == ' ') {
    ++command;
  }

  size_t length = strlen(command);

  while (
      length > 0 &&
      command[length - 1] == ' '
  ) {
    command[length - 1] = '\0';
    --length;
  }
}

void handleCommand(char* command) {
  trimCommand(command);
  lastHostContactMs = millis();

  if (*command == '\0') {
    return;
  }

  if (strcmp(command, "HELLO") == 0) {
    Serial.println(
        "ACK HELLO firmware=shred-interactive-1.0"
    );
    return;
  }

  if (strcmp(command, "PING") == 0) {
    Serial.println("PONG");
    return;
  }

  if (strcmp(command, "GET STATUS") == 0) {
    printStatus();
    return;
  }

  if (strcmp(command, "GET CONFIG") == 0) {
    printConfig();
    return;
  }

  if (strcmp(command, "START STREAM") == 0) {
    streamingEnabled = true;
    lastTelemetryUs = micros();
    Serial.println("ACK START STREAM");
    return;
  }

  if (strcmp(command, "STOP STREAM") == 0) {
    streamingEnabled = false;
    Serial.println("ACK STOP STREAM");
    return;
  }

  if (strcmp(command, "ZERO") == 0) {
    if (
        deviceState != STATE_IDLE &&
        deviceState != STATE_ARMED
    ) {
      Serial.println(
          "NACK ZERO reason=device_busy"
      );
      return;
    }

    zeroEncoderCounts();

    Serial.println(
        "ACK ZERO encoderCounts=0 basketDeg=0.0000"
    );
    return;
  }

  if (strcmp(command, "ARM") == 0) {
    if (
        deviceState == STATE_FAULT ||
        deviceState == STATE_ESTOP ||
        systemIsRunning()
    ) {
      Serial.println(
          "NACK ARM reason=invalid_state"
      );
      return;
    }

    stopAllOutputs();
    deviceState = STATE_ARMED;

    strncpy(
        faultReason,
        "NONE",
        sizeof(faultReason) - 1
    );
    faultReason[sizeof(faultReason) - 1] = '\0';

    Serial.println("ACK ARM state=ARMED");
    return;
  }

  if (strcmp(command, "START") == 0) {
    if (deviceState != STATE_ARMED) {
      Serial.println(
          "NACK START reason=not_armed"
      );
      return;
    }

    Serial.println("ACK START");
    startContinuousOscillation();
    return;
  }

  if (strcmp(command, "STOP") == 0) {
    stopAllOutputs();

    waveformPending = false;
    pendingTrialId = -1;

    deviceState = STATE_IDLE;
    currentTargetDeg = currentPosDeg;

    Serial.println("ACK STOP state=IDLE");
    return;
  }

  if (strcmp(command, "ESTOP") == 0) {
    enterEstop();
    Serial.println("ACK ESTOP");
    return;
  }

  if (strcmp(command, "CLEAR FAULT") == 0) {
    if (
        deviceState != STATE_FAULT &&
        deviceState != STATE_ESTOP
    ) {
      Serial.println(
          "NACK CLEAR FAULT reason=no_fault_latched"
      );
      return;
    }

    stopAllOutputs();

    waveformPending = false;
    pendingTrialId = -1;

    deviceState = STATE_IDLE;

    strncpy(
        faultReason,
        "NONE",
        sizeof(faultReason) - 1
    );
    faultReason[sizeof(faultReason) - 1] = '\0';

    Serial.println(
        "ACK CLEAR FAULT state=IDLE"
    );
    return;
  }

  if (strcmp(command, "CANCEL WAVEFORM") == 0) {
    if (waveformActive) {
      Serial.println(
          "NACK CANCEL WAVEFORM reason=waveform_already_active"
      );
      return;
    }

    waveformPending = false;
    pendingTrialId = -1;
    pendingDirection = 0;

    Serial.println(
        "ACK CANCEL WAVEFORM"
    );
    return;
  }

  if (
      strncmp(
          command,
          "QUEUE WAVEFORM ",
          15
      ) == 0
  ) {
    handleQueueWaveformCommand(command);
    return;
  }

  if (strncmp(command, "SET ", 4) == 0) {
    handleSetCommand(command);
    return;
  }

  Serial.print(
      "NACK reason=unknown_command command="
  );
  Serial.println(command);
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      rxBuffer[rxLength] = '\0';

      if (rxLength > 0) {
        char* command = rxBuffer;
        handleCommand(command);
      }

      rxLength = 0;
      continue;
    }

    if (rxLength < RX_BUFFER_SIZE - 1) {
      rxBuffer[rxLength++] = incoming;
    } else {
      rxLength = 0;
      Serial.println(
          "NACK reason=command_too_long"
      );
    }
  }
}

// Setup / loop
void setup() {
  pinMode(MOTOR_PIN_PWM, OUTPUT);
  pinMode(MOTOR_PIN_DIR, OUTPUT);
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  pinMode(MAG_PIN_PWM, OUTPUT);
  pinMode(MAG_PIN_DIR, OUTPUT);

  analogWriteFrequency(MOTOR_PIN_PWM, 20000);
  analogWriteFrequency(MAG_PIN_PWM, 20000);

  stopAllOutputs();

  digitalWriteFast(MOTOR_PIN_DIR, LOW);
  digitalWriteFast(MAG_PIN_DIR, LOW);

  attachInterrupt(
      digitalPinToInterrupt(PIN_ENC_A),
      isrEncA,
      CHANGE
  );

  attachInterrupt(
      digitalPinToInterrupt(PIN_ENC_B),
      isrEncB,
      CHANGE
  );

  Serial.begin(115200);

  unsigned long waitStartMs = millis();

  while (
      !Serial &&
      millis() - waitStartMs < 2000UL
  ) {
  }

  zeroEncoderCounts();
  lastHostContactMs = millis();

  Serial.println(
      "READY firmware=shred-interactive-1.0 state=IDLE"
  );
}

void loop() {
  readSerialCommands();
  updateKinematics();

  if (
      systemIsRunning() &&
      millis() - lastHostContactMs >
      HOST_TIMEOUT_MS
  ) {
    enterFault("HOST_TIMEOUT");
  }

  if (isMovingState()) {
    updateMotorControl();
  }
  else if (deviceState == STATE_DWELLING) {
    updateDwellState();
  }
  else {
    stopMotor();

    if (!waveformActive) {
      stopMagProcess();
    }
  }

  sendTelemetryWhenDue();
}