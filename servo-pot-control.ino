#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Define servo channels
#define BASE_CHANNEL     0  // continuous‐rotation servo
#define JOINT1_CHANNEL   1  // 180° positional servo
#define JOINT2_CHANNEL   2  // 180° positional servo
#define GRIPPER_CHANNEL  3  // 180° positional servo

// Define potentiometer pins
#define POT_BASE     A0
#define POT_JOINT1   A1
#define POT_JOINT2   A2
#define POT_GRIPPER  A3

// Servo pulse width limits (microseconds)
const int BASE_STOP  = 1500;  // neutral pulse for “stop”
const int BASE_SPEED =  400;  // speed range around neutral (±400 µs)
const int JOINT_MIN  = 1000;  // 180° servo minimum pulse
const int JOINT_MAX  = 2000;  // 180° servo maximum pulse

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // analog servos run at ~50 Hz
  delay(10);
}

void loop() {
  // ----- BASE (continuous‐rotation) -----
  int potBase = analogRead(POT_BASE);              // 0–1023
  int baseOffset = potBase - 512;                   // -512…+511
  // Map full pot range to ±BASE_SPEED around BASE_STOP
  int basePulse = map(baseOffset, -512, +511,
                      BASE_STOP - BASE_SPEED, 
                      BASE_STOP + BASE_SPEED);
  pwm.writeMicroseconds(BASE_CHANNEL, basePulse);

  // ----- JOINT 1 & 2 and GRIPPER (positional) -----
  int potJoint1  = analogRead(POT_JOINT1);
  int potJoint2  = analogRead(POT_JOINT2);
  int potGripper = analogRead(POT_GRIPPER);

  int joint1Pulse  = map(potJoint1,   0, 1023, JOINT_MIN, JOINT_MAX);
  int joint2Pulse  = map(potJoint2,   0, 1023, JOINT_MIN, JOINT_MAX);
  int gripperPulse = map(potGripper,  0, 1023, JOINT_MIN, JOINT_MAX);

  pwm.writeMicroseconds(JOINT1_CHANNEL,  joint1Pulse);
  pwm.writeMicroseconds(JOINT2_CHANNEL,  joint2Pulse);
  pwm.writeMicroseconds(GRIPPER_CHANNEL, gripperPulse);

  delay(20);
}
