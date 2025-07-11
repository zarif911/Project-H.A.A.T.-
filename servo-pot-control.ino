#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Define servo channelss
#define BASE_CHANNEL 0
#define JOINT1_CHANNEL 1
#define JOINT2_CHANNEL 2
#define GRIPPER_CHANNEL 3

// Define potentiometer pins
#define POT_BASE A0
#define POT_JOINT1 A1
#define POT_JOINT2 A2
#define POT_GRIPPER A3

// Servo pulse width limits (microseconds)
const int BASE_MIN = 500;    // 360° servo minimum pulse
const int BASE_MAX = 2500;   // 360° servo maximum pulse
const int JOINT_MIN = 1000;  // 180° servo minimum pulse
const int JOINT_MAX = 2000;  // 180° servo maximum pulse

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz
  delay(10);
}

void loop() {
  // Read potentiometer values
  int potBase = analogRead(POT_BASE);
  int potJoint1 = analogRead(POT_JOINT1);
  int potJoint2 = analogRead(POT_JOINT2);
  int potGripper = analogRead(POT_GRIPPER);
  
  // Map potentiometer values to servo pulse widths
  int basePulse = map(potBase, 0, 1023, BASE_MIN, BASE_MAX);
  int joint1Pulse = map(potJoint1, 0, 1023, JOINT_MIN, JOINT_MAX);
  int joint2Pulse = map(potJoint2, 0, 1023, JOINT_MIN, JOINT_MAX);
  int gripperPulse = map(potGripper, 0, 1023, JOINT_MIN, JOINT_MAX);
  
  // Send commands to servos
  pwm.writeMicroseconds(BASE_CHANNEL, basePulse);
  pwm.writeMicroseconds(JOINT1_CHANNEL, joint1Pulse);
  pwm.writeMicroseconds(JOINT2_CHANNEL, joint2Pulse);
  pwm.writeMicroseconds(GRIPPER_CHANNEL, gripperPulse);
  
  // Add small delay for stability
  delay(20);
}