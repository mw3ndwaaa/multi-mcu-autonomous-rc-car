/*
  Multi-MCU Autonomous RC Car - Arduino Mega 2560 Version
  --------------------------------------------------------
  Features:
    - 4x IR obstacle sensors (front-left, front-right, rear-left, rear-right)
    - HC-SR04 ultrasonic sensor on a pan servo
    - MPU6050 yaw feedback for closed-loop turning
    - L298N motor driver controlling left/right motor pairs
    - Buzzer status feedback
    - Hardware Serial1 reserved and enabled for a future Bluetooth module

  IMPORTANT POWER NOTE
  --------------------
  The L298N onboard 5 V regulator is NOT used in this build.
  Remove the L298N 5V-EN jumper and power the L298N logic from a regulated
  external 5 V rail. All grounds must be common.

  Required library:
    MPU6050_light by rfetick

  Arduino Mega 2560 pin map:
    D5  -> L298N ENA
    D6  -> L298N ENB
    D8  -> Buzzer
    D9  -> Pan-servo signal
    D18 -> TX1 -> Bluetooth RX (future)
    D19 -> RX1 <- Bluetooth TX (future)
    D20 -> SDA -> MPU6050 SDA
    D21 -> SCL -> MPU6050 SCL
    D22 -> L298N IN1
    D23 -> L298N IN2
    D24 -> L298N IN3
    D25 -> L298N IN4
    D30 -> IR front-left OUT
    D31 -> IR front-right OUT
    D32 -> IR rear-left OUT
    D33 -> IR rear-right OUT
    D34 -> HC-SR04 TRIG
    D35 -> HC-SR04 ECHO

  Safety design:
    - Robot starts STOPPED after boot.
    - Send 'A' over USB Serial or Bluetooth Serial1 to start autonomous mode.
    - Send 'S' at any time to stop.
    - Front IR sensors can force a reverse while driving forward.
    - Rear IR sensors stop a reverse before the car backs into an obstacle.
    - Turn progress and maximum turn time are monitored.

  Note:
    pulseIn() is still blocking for up to 25 ms per ultrasonic sample.
    The rest of the control architecture is cooperative/non-blocking.
*/

#include <Wire.h>
#include <Servo.h>
#include <MPU6050_light.h>
#include <math.h>

// ============================================================
// CONFIGURATION
// ============================================================

// ---------------- Motor driver: L298N ----------------
const uint8_t PIN_ENA = 5;   // PWM - left motor pair
const uint8_t PIN_ENB = 6;   // PWM - right motor pair
const uint8_t PIN_IN1 = 22;
const uint8_t PIN_IN2 = 23;
const uint8_t PIN_IN3 = 24;
const uint8_t PIN_IN4 = 25;

const uint8_t DRIVE_SPEED       = 180;
const uint8_t REVERSE_SPEED     = 160;
const uint8_t MANUAL_TURN_SPEED = 150;

// Closed-loop turn PWM levels.
// If your motors do not move reliably at TURN_PWM_SLOW, raise it.
const uint8_t TURN_PWM_FAST   = 155;
const uint8_t TURN_PWM_MEDIUM = 135;
const uint8_t TURN_PWM_SLOW   = 120;

// ---------------- IR sensors ----------------
const uint8_t PIN_IR_FRONT_L = 30;
const uint8_t PIN_IR_FRONT_R = 31;
const uint8_t PIN_IR_REAR_L  = 32;
const uint8_t PIN_IR_REAR_R  = 33;

// Verified on this robot: LOW means obstacle.
const bool IR_ACTIVE_LOW = true;

// ---------------- Ultrasonic ----------------
const uint8_t PIN_TRIG = 34;
const uint8_t PIN_ECHO = 35;

const unsigned long ULTRASONIC_TIMEOUT_US = 25000UL;
const unsigned long FORWARD_PING_INTERVAL_MS = 100;

const float DIST_SCAN_TRIGGER_CM = 35.0f;
const float DIST_SAFE_CM         = 40.0f;

// ---------------- Pan servo ----------------
const uint8_t PIN_SERVO_PAN = 9;

// Set this to the command that physically points the HC-SR04 straight ahead.
const int SERVO_CENTER = 90;

// We scan using offsets around the calibrated center.
// Positive offset is assumed to physically point LEFT on this build.
const bool SERVO_POSITIVE_IS_LEFT = true;
const float SERVO_TO_YAW_GAIN = 1.0f;

const uint8_t NUM_SCAN_POSITIONS = 5;
const int SCAN_OFFSETS[NUM_SCAN_POSITIONS] = { +70, +35, 0, -35, -70 };

const unsigned long SCAN_SERVO_SETTLE_MS = 250;
const unsigned long SCAN_SAMPLE_GAP_MS   = 60;
const uint8_t SCAN_SAMPLES_PER_ANGLE     = 3;

// ---------------- MPU6050 turning ----------------
const float TURN_TOLERANCE_DEG = 3.0f;

const unsigned long TURN_PROGRESS_CHECK_MS = 700;
const float TURN_MIN_PROGRESS_DEG = 2.0f;
const unsigned long TURN_TIMEOUT_MS = 5000;

// Verified on this robot:
// LEFT rotation  -> negative yaw
// RIGHT rotation -> positive yaw

// ---------------- Reverse/recovery ----------------
const unsigned long REVERSE_DURATION_MS = 600;

// ---------------- Buzzer ----------------
const uint8_t PIN_BUZZER = 8;

// ---------------- Bluetooth / command interface ----------------
// Mega Serial1 pins:
// TX1 = D18 -> Bluetooth RX
// RX1 = D19 <- Bluetooth TX
const unsigned long BLUETOOTH_BAUD = 9600;

// ---------------- Debugging ----------------
const bool DEBUG_SERIAL = true;
const unsigned long DEBUG_INTERVAL_MS = 500;

// ============================================================
// TYPES / GLOBAL OBJECTS
// ============================================================

enum OperatingMode {
  MODE_STOPPED,
  MODE_AUTO,
  MODE_MANUAL
};

enum CarState {
  STATE_IDLE,
  STATE_FORWARD,
  STATE_SCANNING,
  STATE_TURNING,
  STATE_REVERSING
};

OperatingMode mode = MODE_STOPPED;
CarState state = STATE_IDLE;

Servo panServo;
MPU6050 mpu(Wire);

// ---------------- IR state ----------------
bool irFrontL = false;
bool irFrontR = false;
bool irRearL  = false;
bool irRearR  = false;

// ---------------- Ultrasonic / forward ----------------
unsigned long lastForwardPingMs = 0;
float lastFrontDistanceCm = -1.0f;

// ---------------- Scan bookkeeping ----------------
uint8_t scanStep = 0;
uint8_t scanSampleCount = 0;
uint8_t scanValidSampleCount = 0;
float scanSampleBuffer[SCAN_SAMPLES_PER_ANGLE];
float scanDistances[NUM_SCAN_POSITIONS];
unsigned long scanNextActionMs = 0;
bool scanWaitingForServo = false;

// ---------------- Turning bookkeeping ----------------
float targetYawDeg = 0.0f;
unsigned long turnStartMs = 0;
unsigned long lastTurnProgressCheckMs = 0;
float lastTurnProgressYawDeg = 0.0f;

// ---------------- Reverse bookkeeping ----------------
unsigned long reverseStartMs = 0;

// ---------------- Buzzer bookkeeping ----------------
uint8_t buzzerChirpsRemaining = 0;
bool buzzerIsOn = false;
unsigned long buzzerNextChangeMs = 0;
unsigned int buzzerOnMs = 0;
unsigned int buzzerGapMs = 0;

// ---------------- Debug ----------------
unsigned long lastDebugMs = 0;

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================

void handleUsbCommands();
void handleBluetoothCommands();
void processCommand(char command, Stream &reply);
void printCommandHelp(Stream &out);
void handleAutoSafety();
void handleForward();
void beginScan();
void handleScanning();
void finishScanAndChooseDirection();
void beginTurn(float relativeYawDeg);
void handleTurning();
void startReverse(bool recoveryFromFailedTurn);
void handleReversing();
void updateIRStates();
bool readIR(uint8_t pin);
float readUltrasonicSingleCm();
float robustScanDistance();
void movePanToScanStep(uint8_t step);
float servoOffsetToYawDelta(int servoOffset);
uint8_t chooseTurnPwm(float absErrorDeg);
void resetTurnProgressTracking();
void driveForward(uint8_t speedValue);
void driveReverse(uint8_t speedValue);
void turnLeftInPlace(uint8_t speedValue);
void turnRightInPlace(uint8_t speedValue);
void stopMotors();
void emergencyStop();
void startChirp(uint8_t chirps, unsigned int onMs, unsigned int gapMs);
void updateBuzzer();
void printDebugLine();
const char* modeName(OperatingMode value);
const char* stateName(CarState value);

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial1.begin(BLUETOOTH_BAUD);

  // Motor driver
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  stopMotors();

  // IR sensors
  pinMode(PIN_IR_FRONT_L, INPUT);
  pinMode(PIN_IR_FRONT_R, INPUT);
  pinMode(PIN_IR_REAR_L, INPUT);
  pinMode(PIN_IR_REAR_R, INPUT);

  // Ultrasonic
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // Pan servo
  panServo.attach(PIN_SERVO_PAN);
  panServo.write(SERVO_CENTER);

  // MPU6050 / I2C
  Wire.begin();
  Wire.setClock(100000);

  #ifdef WIRE_HAS_TIMEOUT
    Wire.setWireTimeout(25000UL, true);
  #endif

  byte status = mpu.begin();

  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  if (status != 0) {
    Serial.println(F("MPU6050 connection failed. Check power/SDA/SCL."));
    while (true) {
      stopMotors();
      digitalWrite(PIN_BUZZER, HIGH);
      delay(100);
      digitalWrite(PIN_BUZZER, LOW);
      delay(900);
    }
  }

  Serial.println(F("Keep the car completely still. Calibrating MPU6050..."));
  delay(1000);
  mpu.calcOffsets();

  stopMotors();
  mode = MODE_STOPPED;
  state = STATE_IDLE;

  startChirp(2, 80, 80);
  printCommandHelp(Serial);

  Serial.println(F("System ready. Robot is STOPPED until command 'A'."));
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  mpu.update();

  updateBuzzer();
  updateIRStates();
  handleUsbCommands();
  handleBluetoothCommands();

  if (mode == MODE_AUTO) {
    handleAutoSafety();

    switch (state) {
      case STATE_FORWARD:   handleForward();   break;
      case STATE_SCANNING:  handleScanning();  break;
      case STATE_TURNING:   handleTurning();   break;
      case STATE_REVERSING: handleReversing(); break;
      case STATE_IDLE:
      default:
        stopMotors();
        break;
    }
  }

  if (DEBUG_SERIAL && millis() - lastDebugMs >= DEBUG_INTERVAL_MS) {
    lastDebugMs = millis();
    printDebugLine();
  }
}

// ============================================================
// COMMAND INTERFACE
// ============================================================

void handleUsbCommands() {
  while (Serial.available()) {
    char command = Serial.read();
    processCommand(command, Serial);
  }
}

void handleBluetoothCommands() {
  while (Serial1.available()) {
    char command = Serial1.read();
    processCommand(command, Serial1);
  }
}

void processCommand(char command, Stream &reply) {
  if (command == '\r' || command == '\n' || command == ' ') return;

  if (command >= 'a' && command <= 'z') {
    command = command - ('a' - 'A');
  }

  switch (command) {
    case 'A':
      stopMotors();
      panServo.write(SERVO_CENTER);
      mode = MODE_AUTO;
      state = STATE_FORWARD;
      resetTurnProgressTracking();
      startChirp(1, 100, 80);
      reply.println(F("AUTO mode started."));
      break;

    case 'S':
      emergencyStop();
      reply.println(F("STOPPED."));
      break;

    case 'F':
      mode = MODE_MANUAL;
      state = STATE_IDLE;
      if (irFrontL || irFrontR) {
        stopMotors();
        reply.println(F("Forward blocked by front IR sensor."));
      } else {
        driveForward(DRIVE_SPEED);
        reply.println(F("Manual FORWARD."));
      }
      break;

    case 'B':
      mode = MODE_MANUAL;
      state = STATE_IDLE;
      if (irRearL || irRearR) {
        stopMotors();
        reply.println(F("Reverse blocked by rear IR sensor."));
      } else {
        driveReverse(REVERSE_SPEED);
        reply.println(F("Manual REVERSE."));
      }
      break;

    case 'L':
      mode = MODE_MANUAL;
      state = STATE_IDLE;
      turnLeftInPlace(MANUAL_TURN_SPEED);
      reply.println(F("Manual LEFT turn."));
      break;

    case 'R':
      mode = MODE_MANUAL;
      state = STATE_IDLE;
      turnRightInPlace(MANUAL_TURN_SPEED);
      reply.println(F("Manual RIGHT turn."));
      break;

    case 'C':
      panServo.write(SERVO_CENTER);
      reply.println(F("Pan servo centered."));
      break;

    case '?':
    case 'H':
      printCommandHelp(reply);
      break;

    default:
      reply.print(F("Unknown command: "));
      reply.println(command);
      break;
  }
}

void printCommandHelp(Stream &out) {
  out.println();
  out.println(F("Commands:"));
  out.println(F("  A = autonomous mode"));
  out.println(F("  S = stop"));
  out.println(F("  F = manual forward"));
  out.println(F("  B = manual reverse"));
  out.println(F("  L = manual left"));
  out.println(F("  R = manual right"));
  out.println(F("  C = center pan servo"));
  out.println(F("  H or ? = help"));
  out.println();
}

// ============================================================
// AUTONOMOUS SAFETY
// ============================================================

void handleAutoSafety() {
  bool frontBlocked = irFrontL || irFrontR;
  bool rearBlocked  = irRearL  || irRearR;

  // Driving forward: front IR is a close-range emergency trigger.
  if (state == STATE_FORWARD && frontBlocked) {
    startReverse(false);
    return;
  }

  // Reversing: never continue backing toward a rear obstacle.
  if (state == STATE_REVERSING && rearBlocked) {
    stopMotors();
    beginScan();
    return;
  }

  // During an in-place turn, a close obstacle on any corner means stop
  // the turn and reassess. If only the front is blocked and the rear is
  // free, create space by backing up first.
  if (state == STATE_TURNING && (frontBlocked || rearBlocked)) {
    if (frontBlocked && !rearBlocked) {
      startReverse(false);
    } else {
      stopMotors();
      beginScan();
    }
  }
}

// ============================================================
// STATE HANDLERS
// ============================================================

void handleForward() {
  driveForward(DRIVE_SPEED);

  if (millis() - lastForwardPingMs < FORWARD_PING_INTERVAL_MS) return;
  lastForwardPingMs = millis();

  float distance = readUltrasonicSingleCm();
  lastFrontDistanceCm = distance;

  if (distance > 0.0f && distance <= DIST_SCAN_TRIGGER_CM) {
    stopMotors();
    beginScan();
  }
}

void beginScan() {
  stopMotors();
  state = STATE_SCANNING;

  scanStep = 0;
  scanSampleCount = 0;
  scanValidSampleCount = 0;

  for (uint8_t i = 0; i < NUM_SCAN_POSITIONS; i++) {
    scanDistances[i] = -1.0f;
  }

  movePanToScanStep(scanStep);
  scanWaitingForServo = true;
  scanNextActionMs = millis() + SCAN_SERVO_SETTLE_MS;
}

void handleScanning() {
  if (millis() < scanNextActionMs) return;

  if (scanWaitingForServo) {
    scanWaitingForServo = false;
    scanSampleCount = 0;
    scanValidSampleCount = 0;
    scanNextActionMs = millis();
    return;
  }

  float d = readUltrasonicSingleCm();

  if (d > 0.0f && scanValidSampleCount < SCAN_SAMPLES_PER_ANGLE) {
    scanSampleBuffer[scanValidSampleCount] = d;
    scanValidSampleCount++;
  }

  scanSampleCount++;

  if (scanSampleCount < SCAN_SAMPLES_PER_ANGLE) {
    scanNextActionMs = millis() + SCAN_SAMPLE_GAP_MS;
    return;
  }

  scanDistances[scanStep] = robustScanDistance();
  scanStep++;

  if (scanStep >= NUM_SCAN_POSITIONS) {
    finishScanAndChooseDirection();
    return;
  }

  movePanToScanStep(scanStep);
  scanWaitingForServo = true;
  scanNextActionMs = millis() + SCAN_SERVO_SETTLE_MS;
}

void finishScanAndChooseDirection() {
  panServo.write(SERVO_CENTER);

  int bestIdx = -1;
  float bestDistance = -1.0f;

  for (uint8_t i = 0; i < NUM_SCAN_POSITIONS; i++) {
    if (scanDistances[i] > bestDistance) {
      bestDistance = scanDistances[i];
      bestIdx = i;
    }
  }

  // Every scan position returned invalid/no echo.
  // Fail safely rather than pretending the path is definitely clear.
  if (bestIdx < 0 || bestDistance <= 0.0f) {
    stopMotors();
    mode = MODE_STOPPED;
    state = STATE_IDLE;
    startChirp(4, 100, 100);
    Serial.println(F("SCAN FAILED: no valid ultrasonic readings. Robot stopped."));
    return;
  }

  if (bestDistance < DIST_SAFE_CM) {
    startReverse(false);
    return;
  }

  int servoOffset = SCAN_OFFSETS[bestIdx];
  float requestedYawDelta = servoOffsetToYawDelta(servoOffset);

  // Best path is essentially straight ahead.
  if (fabs(requestedYawDelta) <= TURN_TOLERANCE_DEG) {
    state = STATE_FORWARD;
    return;
  }

  beginTurn(requestedYawDelta);
}

void beginTurn(float relativeYawDeg) {
  stopMotors();

  targetYawDeg = mpu.getAngleZ() + relativeYawDeg;
  state = STATE_TURNING;

  turnStartMs = millis();
  lastTurnProgressCheckMs = turnStartMs;
  lastTurnProgressYawDeg = mpu.getAngleZ();

  startChirp(1, 60, 80);
}

void handleTurning() {
  float currentYaw = mpu.getAngleZ();
  float error = targetYawDeg - currentYaw;
  float absError = fabs(error);

  if (absError <= TURN_TOLERANCE_DEG) {
    stopMotors();
    state = STATE_FORWARD;
    resetTurnProgressTracking();
    return;
  }

  if (millis() - turnStartMs >= TURN_TIMEOUT_MS) {
    startReverse(true);
    return;
  }

  if (millis() - lastTurnProgressCheckMs >= TURN_PROGRESS_CHECK_MS) {
    float progress = fabs(currentYaw - lastTurnProgressYawDeg);

    if (progress < TURN_MIN_PROGRESS_DEG) {
      startReverse(true);
      return;
    }

    lastTurnProgressYawDeg = currentYaw;
    lastTurnProgressCheckMs = millis();
  }

  uint8_t turnPwm = chooseTurnPwm(absError);

  // Verified sign convention:
  // negative error -> turn LEFT
  // positive error -> turn RIGHT
  if (error < 0.0f) {
    turnLeftInPlace(turnPwm);
  } else {
    turnRightInPlace(turnPwm);
  }
}

void startReverse(bool recoveryFromFailedTurn) {
  state = STATE_REVERSING;
  reverseStartMs = millis();

  // Start moving immediately; buzzer runs independently.
  driveReverse(REVERSE_SPEED);

  if (recoveryFromFailedTurn) {
    startChirp(3, 100, 80);
  } else {
    startChirp(2, 70, 70);
  }
}

void handleReversing() {
  if (irRearL || irRearR) {
    stopMotors();
    beginScan();
    return;
  }

  driveReverse(REVERSE_SPEED);

  if (millis() - reverseStartMs >= REVERSE_DURATION_MS) {
    stopMotors();
    beginScan();
  }
}

// ============================================================
// SENSOR HELPERS
// ============================================================

void updateIRStates() {
  irFrontL = readIR(PIN_IR_FRONT_L);
  irFrontR = readIR(PIN_IR_FRONT_R);
  irRearL  = readIR(PIN_IR_REAR_L);
  irRearR  = readIR(PIN_IR_REAR_R);
}

bool readIR(uint8_t pin) {
  int value = digitalRead(pin);
  return IR_ACTIVE_LOW ? (value == LOW) : (value == HIGH);
}

float readUltrasonicSingleCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);

  if (duration == 0) {
    return -1.0f;
  }

  return duration * 0.0343f / 2.0f;
}

float robustScanDistance() {
  if (scanValidSampleCount == 0) {
    return -1.0f;
  }

  if (scanValidSampleCount == 1) {
    return scanSampleBuffer[0];
  }

  if (scanValidSampleCount == 2) {
    return (scanSampleBuffer[0] + scanSampleBuffer[1]) / 2.0f;
  }

  // Median of three valid measurements.
  float a = scanSampleBuffer[0];
  float b = scanSampleBuffer[1];
  float c = scanSampleBuffer[2];

  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }

  return b;
}

// ============================================================
// SCAN / TURN MAPPING
// ============================================================

void movePanToScanStep(uint8_t step) {
  int command = SERVO_CENTER + SCAN_OFFSETS[step];
  command = constrain(command, 0, 180);
  panServo.write(command);
}

float servoOffsetToYawDelta(int servoOffset) {
  // This car's yaw sign is:
  // LEFT = negative, RIGHT = positive.
  // If increasing servo angle points left, positive servo offset must
  // therefore become a negative yaw command.
  float sign = SERVO_POSITIVE_IS_LEFT ? -1.0f : 1.0f;
  return sign * (float)servoOffset * SERVO_TO_YAW_GAIN;
}

uint8_t chooseTurnPwm(float absErrorDeg) {
  if (absErrorDeg > 30.0f) return TURN_PWM_FAST;
  if (absErrorDeg > 12.0f) return TURN_PWM_MEDIUM;
  return TURN_PWM_SLOW;
}

void resetTurnProgressTracking() {
  turnStartMs = 0;
  lastTurnProgressCheckMs = 0;
  lastTurnProgressYawDeg = mpu.getAngleZ();
}

// ============================================================
// MOTOR PRIMITIVES
// ============================================================

void driveForward(uint8_t speedValue) {
  analogWrite(PIN_ENA, speedValue);
  analogWrite(PIN_ENB, speedValue);

  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
}

void driveReverse(uint8_t speedValue) {
  analogWrite(PIN_ENA, speedValue);
  analogWrite(PIN_ENB, speedValue);

  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
}

void turnLeftInPlace(uint8_t speedValue) {
  analogWrite(PIN_ENA, speedValue);
  analogWrite(PIN_ENB, speedValue);

  // Left wheels reverse, right wheels forward.
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
}

void turnRightInPlace(uint8_t speedValue) {
  analogWrite(PIN_ENA, speedValue);
  analogWrite(PIN_ENB, speedValue);

  // Left wheels forward, right wheels reverse.
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
}

void stopMotors() {
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);

  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

void emergencyStop() {
  stopMotors();
  panServo.write(SERVO_CENTER);
  mode = MODE_STOPPED;
  state = STATE_IDLE;
  startChirp(1, 180, 80);
}

// ============================================================
// NON-BLOCKING BUZZER
// ============================================================

void startChirp(uint8_t chirps, unsigned int onMs, unsigned int gapMs) {
  if (chirps == 0) return;

  buzzerChirpsRemaining = chirps;
  buzzerOnMs = onMs;
  buzzerGapMs = gapMs;

  buzzerIsOn = true;
  digitalWrite(PIN_BUZZER, HIGH);
  buzzerNextChangeMs = millis() + buzzerOnMs;
}

void updateBuzzer() {
  if (buzzerChirpsRemaining == 0) return;
  if (millis() < buzzerNextChangeMs) return;

  if (buzzerIsOn) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerIsOn = false;

    buzzerChirpsRemaining--;

    if (buzzerChirpsRemaining == 0) {
      return;
    }

    buzzerNextChangeMs = millis() + buzzerGapMs;
  } else {
    digitalWrite(PIN_BUZZER, HIGH);
    buzzerIsOn = true;
    buzzerNextChangeMs = millis() + buzzerOnMs;
  }
}

// ============================================================
// DEBUG OUTPUT
// ============================================================

void printDebugLine() {
  Serial.print(F("MODE="));
  Serial.print(modeName(mode));

  Serial.print(F(" | STATE="));
  Serial.print(stateName(state));

  Serial.print(F(" | Yaw="));
  Serial.print(mpu.getAngleZ(), 1);

  Serial.print(F(" | IR[FL FR RL RR]="));
  Serial.print(irFrontL);
  Serial.print(' ');
  Serial.print(irFrontR);
  Serial.print(' ');
  Serial.print(irRearL);
  Serial.print(' ');
  Serial.print(irRearR);

  Serial.print(F(" | US="));
  if (lastFrontDistanceCm > 0.0f) {
    Serial.print(lastFrontDistanceCm, 1);
    Serial.print(F("cm"));
  } else {
    Serial.print(F("NA"));
  }

  if (state == STATE_TURNING) {
    Serial.print(F(" | TargetYaw="));
    Serial.print(targetYawDeg, 1);
    Serial.print(F(" | TurnError="));
    Serial.print(targetYawDeg - mpu.getAngleZ(), 1);
  }

  Serial.println();
}

const char* modeName(OperatingMode value) {
  switch (value) {
    case MODE_AUTO:    return "AUTO";
    case MODE_MANUAL:  return "MANUAL";
    case MODE_STOPPED:
    default:           return "STOPPED";
  }
}

const char* stateName(CarState value) {
  switch (value) {
    case STATE_FORWARD:   return "FORWARD";
    case STATE_SCANNING:  return "SCANNING";
    case STATE_TURNING:   return "TURNING";
    case STATE_REVERSING: return "REVERSING";
    case STATE_IDLE:
    default:              return "IDLE";
  }
}
