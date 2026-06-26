/*
 * ============================================================
 *  NLFRC Competition Line Follower
 *  Hardware: ESP32 + TB6612FNG + QTR-8A + 2x N20 500RPM
 * ============================================================
 *  Track features handled:
 *   - Smooth curves (U-turns, loops)
 *   - Sharp zigzag / chevron arrows
 *   - Staircase / step turns
 *   - Crossover diamond sections
 *   - "Stop circle" (broken-line marker detection)
 *   - Finish arch (solid full-sensor blackout)
 *
 *  Controller: PID with adaptive gain switching
 *  Author: Competition-grade template — assign YOUR pins below
 * ============================================================
 */

#include <Arduino.h>
#include <QTRSensors.h>   // Pololu QTR library — install via Library Manager

// ============================================================
//  ██████╗ ██╗███╗   ██╗    ███████╗███████╗████████╗
//  ██╔══██╗██║████╗  ██║    ██╔════╝██╔════╝╚══██╔══╝
//  ██████╔╝██║██╔██╗ ██║    ███████╗█████╗     ██║
//  ██╔═══╝ ██║██║╚██╗██║    ╚════██║██╔══╝     ██║
//  ██║     ██║██║ ╚████║    ███████║███████╗   ██║
//  ╚═╝     ╚═╝╚═╝  ╚═══╝    ╚══════╝╚══════╝   ╚═╝
//  Change ONLY this section — all hardware described here
// ============================================================

// ── QTR-8A Analog Sensor Pins ────────────────────────────────
//  Connect 8 sensor outputs left-to-right (S0=leftmost)
//  Use ESP32 ADC1 pins only: 32,33,34,35,36,39,25,26 (ADC2 conflicts with WiFi)
const uint8_t QTR_PINS[8] = {
  36,   // S0 — far left
  39,   // S1
  34,   // S2
  35,   // S3
  32,   // S4
  33,   // S5
  25,   // S6
  26    // S7 — far right
};

// QTR emitter enable pin (connect to QTR-8A's LEDON pin)
// Set to QTRNoEmitterPin if wired always-on
#define QTR_EMITTER_PIN   27

// ── TB6612FNG Motor Driver Pins ───────────────────────────────
//   LEFT MOTOR  (from driver's perspective, robot facing forward)
#define MOTOR_L_IN1   12    // AIN1
#define MOTOR_L_IN2   14    // AIN2
#define MOTOR_L_PWM   13    // PWMA

//   RIGHT MOTOR
#define MOTOR_R_IN1   26    // BIN1  ← change if conflict with QTR
#define MOTOR_R_IN2   27    // BIN2  ← change if conflict with QTR
#define MOTOR_R_PWM   25    // PWMB  ← change if conflict with QTR

//   TB6612FNG standby pin
#define MOTOR_STBY    15    // HIGH = enabled

// ── Optional: Start / Stop Button ────────────────────────────
#define START_BUTTON_PIN  0    // Boot button on most ESP32 devkits
//   Press once to start, press again to stop

// ── Optional: Status LED ──────────────────────────────────────
#define LED_PIN           2    // Onboard LED — comment out if unused

// ============================================================
//  PID TUNING  (tune AFTER wiring is confirmed)
// ============================================================
// -- Normal / straight + gentle curve --
float KP_NORMAL  = 0.08f;
float KI_NORMAL  = 0.0002f;
float KD_NORMAL  = 1.8f;

// -- Aggressive: zigzags, chevrons, tight turns --
float KP_SHARP   = 0.13f;
float KI_SHARP   = 0.0001f;
float KD_SHARP   = 2.8f;

// -- Base speed of each motor (0-255) --
int BASE_SPEED = 130;          // Start low, increase after PID is stable
int MAX_SPEED  = 200;          // Hard ceiling
int MIN_SPEED  = 0;            // Motors are allowed to stop (no reverse on main drive)

// -- Error threshold to switch between Normal ↔ Sharp PID --
//   QTR-8A position range: 0–7000 (center = 3500)
//   |error| > SHARP_THRESHOLD → switch to aggressive gains
int SHARP_THRESHOLD = 1800;

// ============================================================
//  SPECIAL MARKER DETECTION
// ============================================================
// "Stop Circle": broken line — a gap where ALL sensors read white
// for longer than STOP_GAP_MS triggers a controlled halt.
// Set to 0 to disable.
uint32_t STOP_GAP_MS      = 80;    // ms all-white = stop event
uint32_t STOP_HOLD_MS     = 2000;  // pause duration before shutdown

// "Finish arch": solid black bar — ALL sensors read black for
// longer than FINISH_BAR_MS → robot stops permanently.
uint32_t FINISH_BAR_MS    = 120;

// ============================================================
//  LEDC (PWM) CONFIG  — ESP32 hardware PWM
// ============================================================
#define LEDC_FREQ       20000   // 20 kHz — inaudible to humans, good for N20
#define LEDC_RES        8       // 8-bit (0-255)
#define LEDC_CH_LEFT    0
#define LEDC_CH_RIGHT   1

// ============================================================
//  INTERNAL — do not edit below unless you know what you're doing
// ============================================================
QTRSensors qtr;
uint16_t sensorValues[8];

int   lastError      = 0;
float integral       = 0;
bool  running        = false;
bool  finishReached  = false;

uint32_t allWhiteStart  = 0;
uint32_t allBlackStart  = 0;
bool     allWhiteActive = false;
bool     allBlackActive = false;

// ── Forward declarations ──────────────────────────────────────
void calibrateSensors();
void motorSetup();
void driveMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void motorForward(int ch_pwm, int in1, int in2, int speed);
void motorReverse(int ch_pwm, int in1, int in2, int speed);
void motorBrake(int in1, int in2);
bool allSensorsBlack();
bool allSensorsWhite();
void blinkLED(int times, int ms);
void waitForButton();

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== NLFRC Line Follower BOOT ===");

  // LED
  #ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
  #endif

  // Start button
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  // Motor driver
  motorSetup();
  stopMotors();

  // QTR-8A sensor array
  qtr.setTypeAnalog();
  qtr.setSensorPins(QTR_PINS, 8);
  qtr.setEmitterPin(QTR_EMITTER_PIN);

  Serial.println("Press START button to begin calibration...");
  blinkLED(3, 200);
  waitForButton();

  // Calibrate — sweep robot manually over track line during this phase
  Serial.println("CALIBRATING — move robot over line for 5 seconds...");
  blinkLED(1, 100);

  calibrateSensors();

  Serial.println("Calibration complete. Press START button to run.");
  blinkLED(5, 100);
  waitForButton();

  running      = true;
  finishReached = false;
  lastError    = 0;
  integral     = 0;

  #ifdef LED_PIN
    digitalWrite(LED_PIN, HIGH);
  #endif

  Serial.println("RUNNING!");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  if (!running || finishReached) {
    stopMotors();
    return;
  }

  // ── 1. Read sensors & compute weighted position ───────────
  int position = (int)qtr.readLineBlack(sensorValues);
  // position: 0 (far left) … 7000 (far right), 3500 = center

  // ── 2. Check for special markers ──────────────────────────
  if (allSensorsBlack()) {
    if (!allBlackActive) {
      allBlackActive = true;
      allBlackStart  = millis();
    } else if ((millis() - allBlackStart) >= FINISH_BAR_MS) {
      // FINISH detected
      Serial.println("FINISH LINE — stopping.");
      finishReached = true;
      stopMotors();
      blinkLED(10, 100);
      return;
    }
  } else {
    allBlackActive = false;
  }

  if (STOP_GAP_MS > 0 && allSensorsWhite()) {
    if (!allWhiteActive) {
      allWhiteActive = true;
      allWhiteStart  = millis();
    } else if ((millis() - allWhiteStart) >= STOP_GAP_MS) {
      // STOP CIRCLE detected — pause then continue (or halt, your choice)
      Serial.println("STOP MARKER — pausing.");
      stopMotors();
      delay(STOP_HOLD_MS);
      allWhiteActive = false;
      allBlackActive = false;
      integral       = 0;   // reset integral after pause
      return;
    }
  } else {
    allWhiteActive = false;
  }

  // ── 3. PID computation ────────────────────────────────────
  int error = position - 3500;   // negative = robot drifted left, positive = right

  // Adaptive gain selection
  float kp, ki, kd;
  if (abs(error) > SHARP_THRESHOLD) {
    kp = KP_SHARP;
    ki = KI_SHARP;
    kd = KD_SHARP;
  } else {
    kp = KP_NORMAL;
    ki = KI_NORMAL;
    kd = KD_NORMAL;
  }

  // Anti-windup: clamp integral
  integral += error;
  integral = constrain(integral, -5000, 5000);

  int derivative = error - lastError;
  lastError = error;

  float correction = (kp * error) + (ki * integral) + (kd * derivative);

  // ── 4. Apply correction to motors ─────────────────────────
  int leftSpeed  = BASE_SPEED + (int)correction;
  int rightSpeed = BASE_SPEED - (int)correction;

  leftSpeed  = constrain(leftSpeed,  MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  driveMotors(leftSpeed, rightSpeed);

  // ── 5. Debug output (comment out for competition) ─────────
  // Serial.printf("Pos:%d Err:%d Corr:%.1f L:%d R:%d\n",
  //               position, error, correction, leftSpeed, rightSpeed);
}

// ============================================================
//  SENSOR CALIBRATION
//  Drives robot forward slowly while sampling — or let the
//  user sweep manually if no auto-drive is preferred.
// ============================================================
void calibrateSensors() {
  // Manual sweep mode: just read for 5 seconds
  uint32_t start = millis();
  while (millis() - start < 5000) {
    qtr.calibrate();
    delay(20);

    // Blink LED during calibration
    #ifdef LED_PIN
      digitalWrite(LED_PIN, (millis() / 200) % 2);
    #endif
  }

  #ifdef LED_PIN
    digitalWrite(LED_PIN, LOW);
  #endif

  // Print calibration min/max for debugging
  Serial.print("Cal MIN: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Cal MAX: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(" ");
  }
  Serial.println();
}

// ============================================================
//  MOTOR DRIVER SETUP  (TB6612FNG via ESP32 LEDC PWM)
// ============================================================
void motorSetup() {
  // Configure LEDC channels
  ledcSetup(LEDC_CH_LEFT,  LEDC_FREQ, LEDC_RES);
  ledcSetup(LEDC_CH_RIGHT, LEDC_FREQ, LEDC_RES);

  ledcAttachPin(MOTOR_L_PWM, LEDC_CH_LEFT);
  ledcAttachPin(MOTOR_R_PWM, LEDC_CH_RIGHT);

  // Direction pins
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_STBY,  OUTPUT);

  // Enable TB6612FNG
  digitalWrite(MOTOR_STBY, HIGH);

  // Start stopped
  motorBrake(MOTOR_L_IN1, MOTOR_L_IN2);
  motorBrake(MOTOR_R_IN1, MOTOR_R_IN2);
  ledcWrite(LEDC_CH_LEFT,  0);
  ledcWrite(LEDC_CH_RIGHT, 0);
}

// ── Drive both motors forward at given speeds ─────────────────
void driveMotors(int leftSpeed, int rightSpeed) {
  // Left
  if (leftSpeed >= 0) {
    motorForward(LEDC_CH_LEFT, MOTOR_L_IN1, MOTOR_L_IN2, leftSpeed);
  } else {
    motorReverse(LEDC_CH_LEFT, MOTOR_L_IN1, MOTOR_L_IN2, -leftSpeed);
  }

  // Right
  if (rightSpeed >= 0) {
    motorForward(LEDC_CH_RIGHT, MOTOR_R_IN1, MOTOR_R_IN2, rightSpeed);
  } else {
    motorReverse(LEDC_CH_RIGHT, MOTOR_R_IN1, MOTOR_R_IN2, -rightSpeed);
  }
}

void motorForward(int ch, int in1, int in2, int speed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  ledcWrite(ch, constrain(speed, 0, 255));
}

void motorReverse(int ch, int in1, int in2, int speed) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  ledcWrite(ch, constrain(speed, 0, 255));
}

void motorBrake(int in1, int in2) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
}

void stopMotors() {
  motorBrake(MOTOR_L_IN1, MOTOR_L_IN2);
  motorBrake(MOTOR_R_IN1, MOTOR_R_IN2);
  ledcWrite(LEDC_CH_LEFT,  0);
  ledcWrite(LEDC_CH_RIGHT, 0);
}

// ============================================================
//  MARKER HELPERS
// ============================================================
bool allSensorsBlack() {
  for (int i = 0; i < 8; i++) {
    // readLineBlack already calibrated; raw > 900 = black on white track
    if (sensorValues[i] < 900) return false;
  }
  return true;
}

bool allSensorsWhite() {
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > 100) return false;   // anything > 100 = seeing line
  }
  return true;
}

// ============================================================
//  UTILITIES
// ============================================================
void blinkLED(int times, int ms) {
  #ifdef LED_PIN
    for (int i = 0; i < times; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(ms);
      digitalWrite(LED_PIN, LOW);
      delay(ms);
    }
  #endif
}

void waitForButton() {
  // Debounced button wait — active LOW (pull-up)
  while (digitalRead(START_BUTTON_PIN) == HIGH) { delay(10); }
  delay(50);   // debounce
  while (digitalRead(START_BUTTON_PIN) == LOW)  { delay(10); }
  delay(50);
}
