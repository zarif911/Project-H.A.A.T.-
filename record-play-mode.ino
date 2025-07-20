#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Servo driver setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo channels
#define BASE_CHANNEL     0   // continuous‑rotation
#define JOINT1_CHANNEL   1   // 180° positional
#define JOINT2_CHANNEL   2   // 180° positional
#define GRIPPER_CHANNEL  3   // 180° positional

// Potentiometer pins
#define POT_BASE     A0
#define POT_JOINT1   A1
#define POT_JOINT2   A2
#define POT_GRIPPER  A3

// Button pins
#define RECORD_BTN   22
#define PLAY_BTN     24
#define PAUSE_BTN    26

// LED pins
#define RECORD_LED   28
#define PLAY_LED     30
#define PAUSE_LED    32

// Servo pulse width limits
const int BASE_STOP   = 1500;  // neutral = no movement
const int BASE_SPEED  = 500;   // max speed span ±500µs from stop
const int JOINT_MIN   = 1000;  // 180° min pulse
const int JOINT_MAX   = 2000;  // 180° max pulse

// Recording parameters
#define MAX_STEPS    200
int recordedSequence[MAX_STEPS][4];  // [step][{base,j1,j2,grip}]
int totalSteps   = 0;
int currentStep  = 0;
unsigned long lastStepTime = 0;
const int STEP_INTERVAL = 50;  // ms between frames

// System states
enum State { IDLE, RECORDING, PLAYING, PAUSED };
State currentState = IDLE;

// Button debouncing
unsigned long lastDebounceTime = 0;
const int DEBOUNCE_DELAY = 50;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  // Buttons
  pinMode(RECORD_BTN, INPUT_PULLUP);
  pinMode(PLAY_BTN,   INPUT_PULLUP);
  pinMode(PAUSE_BTN,  INPUT_PULLUP);

  // LEDs
  pinMode(RECORD_LED, OUTPUT);
  pinMode(PLAY_LED,   OUTPUT);
  pinMode(PAUSE_LED,  OUTPUT);

  updateLEDs();
}

void loop() {
  handleButtons();

  switch(currentState) {
    case IDLE:
      manualControl();
      break;
    case RECORDING:
      manualControl();
      recordPosition();
      break;
    case PLAYING:
      playbackSequence();
      break;
    case PAUSED:
      // hold last output; no change
      break;
  }

  updateLEDs();
  delay(20);
}

void handleButtons() {
  if (millis() - lastDebounceTime < DEBOUNCE_DELAY) return;
  lastDebounceTime = millis();

  if (digitalRead(RECORD_BTN) == LOW) {
    if (currentState == RECORDING) {
      currentState = IDLE;
      Serial.print("Recording stopped. Steps = ");
      Serial.println(totalSteps);
    } else {
      totalSteps = 0;
      currentState = RECORDING;
      Serial.println("Recording started");
    }
    delay(200);
    return;
  }

  if (digitalRead(PLAY_BTN) == LOW) {
    if (currentState == PLAYING) {
      // already playing
    } else if (currentState == PAUSED) {
      currentState = PLAYING;
      Serial.println("Playback resumed");
    } else if (totalSteps > 0) {
      currentStep = 0;
      currentState = PLAYING;
      Serial.println("Playback started");
    } else {
      Serial.println("No sequence recorded");
    }
    delay(200);
    return;
  }

  if (digitalRead(PAUSE_BTN) == LOW) {
    if (currentState == PLAYING) {
      currentState = PAUSED;
      Serial.println("Playback paused");
    }
    delay(200);
  }
}

void manualControl() {
  // CONTINUOUS‑ROTATION BASE
  int raw = analogRead(POT_BASE);        // 0–1023
  int offset = raw - 512;                // –512…+511
  int basePulse = map(offset, -512, 511,
                      BASE_STOP - BASE_SPEED,
                      BASE_STOP + BASE_SPEED);

  // POSITIONAL JOINTS
  int j1 = map(analogRead(POT_JOINT1), 0, 1023, JOINT_MIN, JOINT_MAX);
  int j2 = map(analogRead(POT_JOINT2), 0, 1023, JOINT_MIN, JOINT_MAX);
  int grip = map(analogRead(POT_GRIPPER), 0, 1023, JOINT_MIN, JOINT_MAX);

  setServos(basePulse, j1, j2, grip);
}

void recordPosition() {
  if (millis() - lastStepTime < STEP_INTERVAL) return;
  if (totalSteps >= MAX_STEPS) {
    Serial.println("Buffer full!");
    currentState = IDLE;
    return;
  }

  // capture exactly the same mapping as manualControl()
  int raw = analogRead(POT_BASE);
  int offset = raw - 512;
  int basePulse = map(offset, -512, 511,
                      BASE_STOP - BASE_SPEED,
                      BASE_STOP + BASE_SPEED);

  recordedSequence[totalSteps][0] = basePulse;
  recordedSequence[totalSteps][1] = map(analogRead(POT_JOINT1), 0, 1023, JOINT_MIN, JOINT_MAX);
  recordedSequence[totalSteps][2] = map(analogRead(POT_JOINT2), 0, 1023, JOINT_MIN, JOINT_MAX);
  recordedSequence[totalSteps][3] = map(analogRead(POT_GRIPPER),0,1023, JOINT_MIN, JOINT_MAX);

  totalSteps++;
  Serial.print("Recorded step ");
  Serial.println(totalSteps);

  lastStepTime = millis();
}

void playbackSequence() {
  if (millis() - lastStepTime < STEP_INTERVAL) return;

  setServos(
    recordedSequence[currentStep][0],
    recordedSequence[currentStep][1],
    recordedSequence[currentStep][2],
    recordedSequence[currentStep][3]
  );

  currentStep++;
  if (currentStep >= totalSteps) {
    currentStep = 0;
    Serial.println("Playback looped");
  }

  lastStepTime = millis();
}

void setServos(int base, int j1, int j2, int grip) {
  pwm.writeMicroseconds(BASE_CHANNEL,    base);
  pwm.writeMicroseconds(JOINT1_CHANNEL,  j1);
  pwm.writeMicroseconds(JOINT2_CHANNEL,  j2);
  pwm.writeMicroseconds(GRIPPER_CHANNEL, grip);
}

void updateLEDs() {
  digitalWrite(RECORD_LED, currentState == RECORDING);
  digitalWrite(PLAY_LED,   currentState == PLAYING);
  digitalWrite(PAUSE_LED,  currentState == PAUSED);
}
