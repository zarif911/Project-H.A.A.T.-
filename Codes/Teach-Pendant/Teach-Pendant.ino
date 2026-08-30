/*
  Robotic Arm Control with Record/Playback
  Hardware:
    - Arduino Mega
    - PCA9685 Servo Driver (address 0x40)
    - 4 Potentiometers (A0-A3)
    - 3 Pushbuttons (pins 22, 24, 26) with internal pull-up
    - 3 LEDs (pins 28, 30, 32) with 220Ω resistors
    - 4 Servos (channels 0-3)

  Functions:
    - Manual: pots control servos in real-time (default)
    - Record: press REC to start/stop storing positions every 50ms
    - Playback: press PLAY to loop recorded sequence; PAUSE freezes; PLAY resumes
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ============ Pin Definitions ============
// Potentiometers
#define POT_BASE    A0
#define POT_JOINT1  A1
#define POT_JOINT2  A2
#define POT_GRIPPER A3

// Buttons (active LOW, internal pull-up)
#define BUTTON_RECORD 22
#define BUTTON_PLAY   24
#define BUTTON_PAUSE  26

// LEDs (active HIGH)
#define LED_RECORD 28
#define LED_PLAY   30
#define LED_PAUSE  32

// ============ Servo Driver ============
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // default I2C address

// Servo channels (PCA9685 outputs)
#define CH_BASE     0
#define CH_JOINT1   1
#define CH_JOINT2   2
#define CH_GRIPPER  3

// ============ Servo Pulse Ranges ============
// Continuous rotation base: 1500 = stop, 1300 = full CW, 1700 = full CCW
#define BASE_STOP   1500
#define BASE_MIN    1300
#define BASE_MAX    1700

// Standard 180° servos (adjust if needed)
#define JOINT_MIN   500    // pulse width (µs) for 0°
#define JOINT_MAX   2500   // pulse width (µs) for 180°

// ============ Recording/Playback Settings ============
#define MAX_STEPS    200      // maximum number of stored positions
#define STEP_INTERVAL 50      // recording sampling period (ms)
#define PLAY_INTERVAL 50      // playback step interval (ms)
#define DEBOUNCE_DELAY 50     // button debounce time (ms)

// ============ Global State Variables ============
// Recorded sequences
int recordedBase[MAX_STEPS];
int recordedJoint1[MAX_STEPS];
int recordedJoint2[MAX_STEPS];
int recordedGripper[MAX_STEPS];
int stepCount = 0;            // number of stored steps

// Operational modes
bool isRecording = false;
bool isPlaying = false;
bool isPaused = false;
int playIndex = 0;            // current playback position

// Timing
unsigned long lastStepTime = 0;
unsigned long lastPlayTime = 0;
unsigned long lastButtonCheck = 0;

// ============ Utility: Convert µs to PWM ticks ============
uint16_t pulseToTicks(uint16_t pulseUs) {
  // PCA9685: 4096 ticks over 20ms (50Hz)
  // 1 tick = 20000µs / 4096 ≈ 4.8828µs
  return map(pulseUs, 0, 20000, 0, 4096);
}

// ============ Set Servo Pulse ============
void setServoPulse(uint8_t channel, uint16_t pulseUs) {
  pwm.setPWM(channel, 0, pulseToTicks(pulseUs));
}

// ============ Read Potentiometer and Map to Pulse ============
int readPotToPulse(int potPin, int minPulse, int maxPulse) {
  int val = analogRead(potPin);        // 0-1023
  return map(val, 0, 1023, minPulse, maxPulse);
}

// ============ Update Servos from Potentiometers (Manual/Record) ============
void updateServosFromPots() {
  int basePulse   = readPotToPulse(POT_BASE, BASE_MIN, BASE_MAX);
  int joint1Pulse = readPotToPulse(POT_JOINT1, JOINT_MIN, JOINT_MAX);
  int joint2Pulse = readPotToPulse(POT_JOINT2, JOINT_MIN, JOINT_MAX);
  int gripperPulse= readPotToPulse(POT_GRIPPER, JOINT_MIN, JOINT_MAX);

  setServoPulse(CH_BASE, basePulse);
  setServoPulse(CH_JOINT1, joint1Pulse);
  setServoPulse(CH_JOINT2, joint2Pulse);
  setServoPulse(CH_GRIPPER, gripperPulse);
}

// ============ Record Current Position ============
void recordStep() {
  if (stepCount < MAX_STEPS) {
    // Read current pot positions (assume servos already set from pots)
    int basePulse   = readPotToPulse(POT_BASE, BASE_MIN, BASE_MAX);
    int joint1Pulse = readPotToPulse(POT_JOINT1, JOINT_MIN, JOINT_MAX);
    int joint2Pulse = readPotToPulse(POT_JOINT2, JOINT_MIN, JOINT_MAX);
    int gripperPulse= readPotToPulse(POT_GRIPPER, JOINT_MIN, JOINT_MAX);

    recordedBase[stepCount] = basePulse;
    recordedJoint1[stepCount] = joint1Pulse;
    recordedJoint2[stepCount] = joint2Pulse;
    recordedGripper[stepCount] = gripperPulse;
    stepCount++;

    // Optionally serial debug
    // Serial.print("Record step "); Serial.println(stepCount);
  } else {
    // Buffer full – stop recording automatically
    isRecording = false;
    digitalWrite(LED_RECORD, LOW);
    Serial.println("Recording stopped: buffer full");
  }
}

// ============ Play Next Step ============
void playNextStep() {
  if (stepCount == 0) {
    isPlaying = false;
    digitalWrite(LED_PLAY, LOW);
    return;
  }

  // Set servos to the stored values
  setServoPulse(CH_BASE, recordedBase[playIndex]);
  setServoPulse(CH_JOINT1, recordedJoint1[playIndex]);
  setServoPulse(CH_JOINT2, recordedJoint2[playIndex]);
  setServoPulse(CH_GRIPPER, recordedGripper[playIndex]);

  // Advance index; loop continuously
  playIndex++;
  if (playIndex >= stepCount) {
    playIndex = 0;  // loop
  }
}

// ============ Button Handling with Debounce ============
void handleButtons() {
  unsigned long now = millis();
  if (now - lastButtonCheck < DEBOUNCE_DELAY) return;
  lastButtonCheck = now;

  // Read button states (LOW when pressed)
  bool recPressed = (digitalRead(BUTTON_RECORD) == LOW);
  bool playPressed = (digitalRead(BUTTON_PLAY) == LOW);
  bool pausePressed = (digitalRead(BUTTON_PAUSE) == LOW);

  // ----- RECORD button (toggle) -----
  if (recPressed) {
    if (!isRecording) {
      // Start recording: clear buffer
      stepCount = 0;
      isRecording = true;
      digitalWrite(LED_RECORD, HIGH);
      // If playback was active, stop it
      if (isPlaying) {
        isPlaying = false;
        digitalWrite(LED_PLAY, LOW);
        isPaused = false;
        digitalWrite(LED_PAUSE, LOW);
      }
      lastStepTime = millis();  // prepare for first sample
      Serial.println("Recording started");
    } else {
      // Stop recording
      isRecording = false;
      digitalWrite(LED_RECORD, LOW);
      Serial.print("Recording stopped. Steps: ");
      Serial.println(stepCount);
    }
    // Wait a bit to avoid multiple toggles
    delay(DEBOUNCE_DELAY);
    return;
  }

  // ----- PLAY button -----
  if (playPressed) {
    if (isPlaying && !isPaused) {
      // If playing and not paused, stop playback
      isPlaying = false;
      digitalWrite(LED_PLAY, LOW);
      Serial.println("Playback stopped");
    } else if (isPlaying && isPaused) {
      // If paused, resume (unpause)
      isPaused = false;
      digitalWrite(LED_PAUSE, LOW);
      lastPlayTime = millis();  // resume timing
      Serial.println("Playback resumed");
    } else {
      // Start playback (if there are recorded steps)
      if (stepCount > 0) {
        isPlaying = true;
        isPaused = false;
        playIndex = 0;
        digitalWrite(LED_PLAY, HIGH);
        digitalWrite(LED_PAUSE, LOW);
        lastPlayTime = millis();
        // Set first step immediately
        playNextStep();
        Serial.println("Playback started");
      } else {
        Serial.println("No recorded data to play");
      }
    }
    delay(DEBOUNCE_DELAY);
    return;
  }

  // ----- PAUSE button (only if playing) -----
  if (pausePressed) {
    if (isPlaying && !isPaused) {
      isPaused = true;
      digitalWrite(LED_PAUSE, HIGH);
      Serial.println("Playback paused");
    }
    // If already paused, do nothing (or optionally toggle off – but spec says PLAY resumes)
    delay(DEBOUNCE_DELAY);
    return;
  }
}

// ============ Setup ============
void setup() {
  Serial.begin(9600);
  Serial.println("Robotic Arm Controller started");

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);  // 50 Hz for servos
  delay(10);

  // Set servo pulse to safe defaults (stop base, servos centered)
  setServoPulse(CH_BASE, BASE_STOP);
  setServoPulse(CH_JOINT1, (JOINT_MIN + JOINT_MAX) / 2);
  setServoPulse(CH_JOINT2, (JOINT_MIN + JOINT_MAX) / 2);
  setServoPulse(CH_GRIPPER, (JOINT_MIN + JOINT_MAX) / 2);

  // Configure button pins with internal pull-up
  pinMode(BUTTON_RECORD, INPUT_PULLUP);
  pinMode(BUTTON_PLAY, INPUT_PULLUP);
  pinMode(BUTTON_PAUSE, INPUT_PULLUP);

  // LED pins as outputs
  pinMode(LED_RECORD, OUTPUT);
  pinMode(LED_PLAY, OUTPUT);
  pinMode(LED_PAUSE, OUTPUT);

  digitalWrite(LED_RECORD, LOW);
  digitalWrite(LED_PLAY, LOW);
  digitalWrite(LED_PAUSE, LOW);

  // Initialize timing variables
  lastStepTime = millis();
  lastPlayTime = millis();
  lastButtonCheck = millis();
}

// ============ Main Loop ============
void loop() {
  handleButtons();

  // ----- Recording mode -----
  if (isRecording) {
    // In recording, we still want to move the arm manually via pots
    updateServosFromPots();

    // Record a step every STEP_INTERVAL
    if (millis() - lastStepTime >= STEP_INTERVAL) {
      recordStep();
      lastStepTime = millis();
    }
    return; // skip other modes
  }

  // ----- Playback mode -----
  if (isPlaying) {
    if (!isPaused) {
      // Play a new step at PLAY_INTERVAL
      if (millis() - lastPlayTime >= PLAY_INTERVAL) {
        playNextStep();
        lastPlayTime = millis();
      }
    }
    // If paused, do nothing (servos stay at last set position)
    return;
  }

  // ----- Manual mode (default) -----
  updateServosFromPots();
}