#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Servo driver setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo channels
#define BASE_CHANNEL 0
#define JOINT1_CHANNEL 1
#define JOINT2_CHANNEL 2
#define GRIPPER_CHANNEL 3

// Potentiometer pins
#define POT_BASE A0
#define POT_JOINT1 A1
#define POT_JOINT2 A2
#define POT_GRIPPER A3

// Button pins
#define RECORD_BTN 22
#define PLAY_BTN 24
#define PAUSE_BTN 26

// LED pins
#define RECORD_LED 28
#define PLAY_LED 30
#define PAUSE_LED 32

// Servo pulse width limits
const int BASE_MIN = 500;    // 360° servo min pulse
const int BASE_MAX = 2500;   // 360° servo max pulse
const int JOINT_MIN = 1000;  // 180° servo min pulse
const int JOINT_MAX = 2000;  // 180° servo max pulse

// Recording parameters
#define MAX_STEPS 200
int recordedSequence[MAX_STEPS][4];  // Stores servo positions
int totalSteps = 0;                 // Number of recorded steps
int currentStep = 0;                // Current playback step
unsigned long lastStepTime = 0;
const int STEP_INTERVAL = 50;  // Time between steps (ms)

// System states
enum State { IDLE, RECORDING, PLAYING, PAUSED };
State currentState = IDLE;

// Button debouncing
unsigned long lastDebounceTime = 0;
const int DEBOUNCE_DELAY = 50;

void setup() {
  Serial.begin(9600);
  
  // Initialize PWM driver
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  
  // Setup buttons with pull-up resistors
  pinMode(RECORD_BTN, INPUT_PULLUP);
  pinMode(PLAY_BTN, INPUT_PULLUP);
  pinMode(PAUSE_BTN, INPUT_PULLUP);
  
  // Setup LEDs
  pinMode(RECORD_LED, OUTPUT);
  pinMode(PLAY_LED, OUTPUT);
  pinMode(PAUSE_LED, OUTPUT);
  
  // Initialize LEDs
  updateLEDs();
}

void loop() {
  // Handle button presses with debouncing
  handleButtons();
  
  // Process based on current state
  switch(currentState) {
    case IDLE:
      // Manual control using potentiometers
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
      // Maintain current position
      break;
  }
  
  // Update LED indicators
  updateLEDs();
  
  // Small delay for stability
  delay(20);
}

void handleButtons() {
  // Debounce logic
  if ((millis() - lastDebounceTime) < DEBOUNCE_DELAY) return;
  lastDebounceTime = millis();
  
  // Record button pressed
  if (digitalRead(RECORD_BTN) == LOW) {
    if (currentState == RECORDING) {
      // Stop recording
      currentState = IDLE;
      Serial.println("Recording stopped. Steps recorded: " + String(totalSteps));
    } else {
      // Start new recording
      currentState = RECORDING;
      totalSteps = 0;
      Serial.println("Recording started");
    }
    delay(200); // Prevent multiple triggers
    return;
  }
  
  // Play button pressed
  if (digitalRead(PLAY_BTN) == LOW) {
    if (currentState == PLAYING) {
      // Already playing - do nothing
    } else if (currentState == PAUSED) {
      // Resume playback
      currentState = PLAYING;
      Serial.println("Playback resumed");
    } else if (totalSteps > 0) {
      // Start playback
      currentState = PLAYING;
      currentStep = 0;
      Serial.println("Playback started");
    } else {
      Serial.println("No sequence recorded!");
    }
    delay(200);
    return;
  }
  
  // Pause button pressed
  if (digitalRead(PAUSE_BTN) == LOW) {
    if (currentState == PLAYING) {
      currentState = PAUSED;
      Serial.println("Playback paused");
    }
    delay(200);
    return;
  }
}

void manualControl() {
  // Read potentiometers
  int potBase = analogRead(POT_BASE);
  int potJoint1 = analogRead(POT_JOINT1);
  int potJoint2 = analogRead(POT_JOINT2);
  int potGripper = analogRead(POT_GRIPPER);
  
  // Map to servo pulse widths
  int basePulse = map(potBase, 0, 1023, BASE_MIN, BASE_MAX);
  int joint1Pulse = map(potJoint1, 0, 1023, JOINT_MIN, JOINT_MAX);
  int joint2Pulse = map(potJoint2, 0, 1023, JOINT_MIN, JOINT_MAX);
  int gripperPulse = map(potGripper, 0, 1023, JOINT_MIN, JOINT_MAX);
  
  // Send to servos
  setServos(basePulse, joint1Pulse, joint2Pulse, gripperPulse);
}

void recordPosition() {
  // Only record at specified intervals
  if ((millis() - lastStepTime) < STEP_INTERVAL) return;
  
  if (totalSteps < MAX_STEPS) {
    // Read current positions
    int basePulse = map(analogRead(POT_BASE), 0, 1023, BASE_MIN, BASE_MAX);
    int joint1Pulse = map(analogRead(POT_JOINT1), 0, 1023, JOINT_MIN, JOINT_MAX);
    int joint2Pulse = map(analogRead(POT_JOINT2), 0, 1023, JOINT_MIN, JOINT_MAX);
    int gripperPulse = map(analogRead(POT_GRIPPER), 0, 1023, JOINT_MIN, JOINT_MAX);
    
    // Store in sequence
    recordedSequence[totalSteps][0] = basePulse;
    recordedSequence[totalSteps][1] = joint1Pulse;
    recordedSequence[totalSteps][2] = joint2Pulse;
    recordedSequence[totalSteps][3] = gripperPulse;
    
    totalSteps++;
    Serial.println("Recorded step: " + String(totalSteps));
    lastStepTime = millis();
  } else {
    Serial.println("Recording buffer full!");
    currentState = IDLE;
  }
}

void playbackSequence() {
  // Only advance at specified intervals
  if ((millis() - lastStepTime) < STEP_INTERVAL) return;
  
  // Set servos to current step position
  setServos(
    recordedSequence[currentStep][0],
    recordedSequence[currentStep][1],
    recordedSequence[currentStep][2],
    recordedSequence[currentStep][3]
  );
  
  // Move to next step
  currentStep++;
  
  // Loop back to start when sequence ends
  if (currentStep >= totalSteps) {
    currentStep = 0;
    Serial.println("Playback looped");
  }
  
  lastStepTime = millis();
}

void setServos(int base, int joint1, int joint2, int gripper) {
  pwm.writeMicroseconds(BASE_CHANNEL, base);
  pwm.writeMicroseconds(JOINT1_CHANNEL, joint1);
  pwm.writeMicroseconds(JOINT2_CHANNEL, joint2);
  pwm.writeMicroseconds(GRIPPER_CHANNEL, gripper);
}

void updateLEDs() {
  // Update LEDs based on current state
  digitalWrite(RECORD_LED, currentState == RECORDING);
  digitalWrite(PLAY_LED, currentState == PLAYING);
  digitalWrite(PAUSE_LED, currentState == PAUSED);
}
