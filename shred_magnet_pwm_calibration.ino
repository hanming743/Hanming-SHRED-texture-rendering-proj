/*
  SHRED Magnet PWM Calibration
  Temporary firmware for measuring magnetic field strength vs PWM.

  Hardware mapping:
    Motor PWM : pin 9
    Motor DIR : pin 8
    Magnet PWM: pin 10
    Magnet DIR: pin 7

  Motor is always held at PWM 0.
  Magnet starts at PWM 0.
  Magnet DIR is held HIGH to match the experiment firmware.
  PWM frequency is 20 kHz.

  Serial commands at 115200 baud:
    0 ... 255   -> set magnet PWM
    off         -> set magnet PWM to 0
    status      -> print current PWM
    help        -> print commands
*/

#include <Arduino.h>

const uint8_t MOTOR_PIN_PWM = 9;
const uint8_t MOTOR_PIN_DIR = 8;
const uint8_t MAG_PIN_PWM   = 10;
const uint8_t MAG_PIN_DIR   = 7;

const uint32_t PWM_FREQUENCY_HZ = 20000;

int currentMagPwm = 0;
String inputLine;

void setMotorSafeOff() {
  analogWrite(MOTOR_PIN_PWM, 0);
  digitalWriteFast(MOTOR_PIN_DIR, LOW);
}

void setMagnetPwm(int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWriteFast(MAG_PIN_DIR, HIGH);
  analogWrite(MAG_PIN_PWM, pwm);
  currentMagPwm = pwm;
}

void printHelp() {
  Serial.println();
  Serial.println("SHRED MAGNET PWM CALIBRATION");
  Serial.println("Commands:");
  Serial.println("  0 ... 255 : set magnet PWM");
  Serial.println("  off       : set magnet PWM to 0");
  Serial.println("  status    : show current PWM");
  Serial.println("  help      : show this message");
  Serial.println();
}

void processCommand(String command) {
  command.trim();
  command.toLowerCase();

  if (command.length() == 0) return;

  if (command == "off") {
    setMagnetPwm(0);
    Serial.println("MAG_PWM=0");
    return;
  }

  if (command == "status") {
    Serial.print("MAG_PWM=");
    Serial.println(currentMagPwm);
    return;
  }

  if (command == "help") {
    printHelp();
    return;
  }

  bool validInteger = true;
  int startIndex = 0;

  if (command[0] == '+' || command[0] == '-') {
    startIndex = 1;
  }

  if (startIndex >= (int)command.length()) {
    validInteger = false;
  }

  for (int i = startIndex; i < (int)command.length(); ++i) {
    if (!isDigit(command[i])) {
      validInteger = false;
      break;
    }
  }

  if (!validInteger) {
    Serial.println("ERROR: enter PWM 0-255, or off/status/help");
    return;
  }

  int pwm = command.toInt();

  if (pwm < 0 || pwm > 255) {
    Serial.println("ERROR: PWM must be between 0 and 255");
    return;
  }

  setMagnetPwm(pwm);

  Serial.print("MAG_PWM=");
  Serial.println(currentMagPwm);
}

void setup() {
  pinMode(MOTOR_PIN_PWM, OUTPUT);
  pinMode(MOTOR_PIN_DIR, OUTPUT);
  pinMode(MAG_PIN_PWM, OUTPUT);
  pinMode(MAG_PIN_DIR, OUTPUT);

  analogWriteFrequency(MOTOR_PIN_PWM, PWM_FREQUENCY_HZ);
  analogWriteFrequency(MAG_PIN_PWM, PWM_FREQUENCY_HZ);

  setMotorSafeOff();
  setMagnetPwm(0);

  Serial.begin(115200);
  delay(300);

  Serial.println("READY: SHRED magnet PWM calibration");
  Serial.println("Motor output is locked at PWM=0.");
  Serial.println("Magnet starts at PWM=0.");
  printHelp();
}

void loop() {
  setMotorSafeOff();

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) {
        processCommand(inputLine);
        inputLine = "";
      }
    } else {
      inputLine += c;

      if (inputLine.length() > 32) {
        inputLine = "";
        Serial.println("ERROR: command too long");
      }
    }
  }
}
