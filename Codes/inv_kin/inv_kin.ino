#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channels
const int SERVO_BASE = 0;
const int SERVO_SHOULDER = 1;
const int SERVO_ELBOW = 2;
const int SERVO_GRIPPER = 3;

const int servoChannels[4] = {SERVO_BASE, SERVO_SHOULDER, SERVO_ELBOW, SERVO_GRIPPER};

// Servo pulse range
#define MIN_PULSE 650
#define MAX_PULSE 2350
#define SERVO_FREQ 50

// Link lengths (mm)
const float L1 = 20.75;  // length of first link
const float L2 = 22.5;   // length of second link

// Recording
#define MAX_RECORDS 500
int recordAngles[MAX_RECORDS][4];
unsigned long recordTimes[MAX_RECORDS];

int recordCount = 0;
bool isRecording = false;
bool isPlaying = false;
unsigned long recordStartTime = 0;
unsigned long lastRecordTime = 0;
unsigned long lastPlayStepTime = 0;
int playIndex = 0;

int lastServoAngles[4] = {0, 0, 0, 0};

// ===== Forward Kinematics =====
// Uses your equations:
// X = (L1 cos θ1 + L2 cos(θ1 - θ2)) cos θ0
// Y = (L1 cos θ1 + L2 cos(θ1 - θ2)) sin θ0
// Z = L1 sin θ1 + L2 sin(θ1 - θ2)
void forwardKinematics(float theta0_deg, float theta1_deg, float theta2_deg,
                        float &X, float &Y, float &Z) {
  float t0 = radians(theta0_deg);
  float t1 = radians(theta1_deg);
  float t2 = radians(theta2_deg);

  X = (L1 * cos(t1) + L2 * cos(t1 - t2)) * cos(t0);
  Y = (L1 * cos(t1) + L2 * cos(t1 - t2)) * sin(t0);
  Z =  L1 * sin(t1) + L2 * sin(t1 - t2);
}

// ===== Set Servo =====
void setServoAngle(int servoIndex, int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  int pulseMicro = map(angle, 0, 180, MIN_PULSE, MAX_PULSE);
  int pulseTicks = int((float)pulseMicro / 1000000 * SERVO_FREQ * 4096);
  pwm.setPWM(servoChannels[servoIndex], 0, pulseTicks);
  lastServoAngles[servoIndex] = angle;

  // After moving servo, compute FK and send position
  float X, Y, Z;
  forwardKinematics(lastServoAngles[SERVO_BASE], lastServoAngles[SERVO_SHOULDER], lastServoAngles[SERVO_ELBOW], X, Y, Z);

  Serial.print("FK Position: X=");
  Serial.print(X);
  Serial.print(" mm, Y=");
  Serial.print(Y);
  Serial.print(" mm, Z=");
  Serial.println(Z);
}

int getServoAngle(int servoIndex) {
  return lastServoAngles[servoIndex];
}

// -------------------- Inverse Kinematics --------------------
// Solves for theta0, theta1, theta2 (degrees) given X,Y,Z (same units as L1,L2).
// elbowUp = true chooses elbow-up solution, false chooses elbow-down.
bool inverseKinematics(float X, float Y, float Z,
                       float &theta0_deg, float &theta1_deg, float &theta2_deg,
                       bool elbowUp = true) {
  // 1) base rotation
  float theta0 = atan2(Y, X);    // radians
  float r = sqrt(X*X + Y*Y);     // planar distance

  // 2) planar (r,Z) problem with substitution theta2' = -theta2
  // Then planar equations become conventional:
  // r = L1 cos θ1 + L2 cos(θ1 + θ2')
  // Z = L1 sin θ1 + L2 sin(θ1 + θ2')
  float R = r;
  float Zp = Z;

  // law of cosines for theta2' (angle between links in conventional form)
  float num = R*R + Zp*Zp - L1*L1 - L2*L2;
  float den = 2.0f * L1 * L2;
  float D = num / den;

  if (D < -1.0f || D > 1.0f) {
    // unreachable
    return false;
  }

  // Two solutions for theta2' (theta2prime): +/- sqrt(1-D^2)
  float sinTerm = sqrt(fmax(0.0f, 1.0f - D*D));
  float sign = elbowUp ? 1.0f : -1.0f;
  float theta2p = atan2(sign * sinTerm, D); // theta2' in radians

  // shoulder theta1 (conventional)
  float theta1 = atan2(Zp, R) - atan2(L2 * sin(theta2p), L1 + L2 * cos(theta2p));

  // convert back: theta2 = -theta2'
  float theta2 = -theta2p;

  // convert to degrees
  theta0_deg = degrees(theta0);
  theta1_deg = degrees(theta1);
  theta2_deg = degrees(theta2);

  // Normalize to -180..180
  while (theta0_deg <= -180.0f) theta0_deg += 360.0f;
  while (theta0_deg >  180.0f) theta0_deg -= 360.0f;
  while (theta1_deg <= -180.0f) theta1_deg += 360.0f;
  while (theta1_deg >  180.0f) theta1_deg -= 360.0f;
  while (theta2_deg <= -180.0f) theta2_deg += 360.0f;
  while (theta2_deg >  180.0f) theta2_deg -= 360.0f;

  return true;
}

// -------------------- Parser and handler --------------------
// Call this when you get a string like "IK X,Y,Z" (X,Y,Z as floats)
void handleIKCommand(String params, bool elbowUp = true) {
  // parse "X,Y,Z" -> floats
  int c1 = params.indexOf(',');
  int c2 = params.indexOf(',', c1 + 1);
  if (c1 == -1 || c2 == -1) {
    Serial.println("Invalid IK format. Use: IK X,Y,Z");
    return;
  }
  float X = params.substring(0, c1).toFloat();
  float Y = params.substring(c1 + 1, c2).toFloat();
  float Z = params.substring(c2 + 1).toFloat();

  float t0_deg, t1_deg, t2_deg;
  bool ok = inverseKinematics(X, Y, Z, t0_deg, t1_deg, t2_deg, elbowUp);

  if (!ok) {
    Serial.println("IK: target unreachable.");
    return;
  }

  // Round and convert to int (servos expect 0-180). Adjust mapping if your servos require offsets.
  int s0 = int(round(t0_deg));
  int s1 = int(round(t1_deg));
  int s2 = int(round(t2_deg));

  // Optional: clamp into 0..180 before sending (or use your calibration system).
  s0 = constrain(s0, 0, 180);
  s1 = constrain(s1, 0, 180);
  s2 = constrain(s2, 0, 180);

  // Apply to servos
  setServoAngle(SERVO_BASE,     s0);
  setServoAngle(SERVO_SHOULDER, s1);
  setServoAngle(SERVO_ELBOW,    s2);

  // report
  Serial.print("IK -> base:"); Serial.print(s0);
  Serial.print(" shoulder:"); Serial.print(s1);
  Serial.print(" elbow:"); Serial.println(s2);

  // show computed FK to verify (uses your forwardKinematics)
  float Xv, Yv, Zv;
  forwardKinematics(s0, s1, s2, Xv, Yv, Zv);
  Serial.print("FK verify X="); Serial.print(Xv);
  Serial.print(" Y="); Serial.print(Yv);
  Serial.print(" Z="); Serial.println(Zv);
}

// ===== Commands =====
void startRecording() {
  if (!isPlaying) {
    isRecording = true;
    recordCount = 0;
    recordStartTime = millis();
    lastRecordTime = recordStartTime;
    Serial.println("Recording started.");
  } else {
    Serial.println("Cannot record while playing.");
  }
}

void stopRecording() {
  if (isRecording) {
    isRecording = false;
    Serial.print("Recording stopped. Total steps: ");
    Serial.println(recordCount);
  } else if (isPlaying) {
    isPlaying = false;
    Serial.println("Playback stopped.");
  }
}

void startPlayback() {
  if (!isRecording && recordCount > 0) {
    isPlaying = true;
    playIndex = 0;
    lastPlayStepTime = millis();
    Serial.println("Playback started.");
  } else if (recordCount == 0) {
    Serial.println("No recorded data to play.");
  } else {
    Serial.println("Cannot play while recording.");
  }
}

void handleAnglesCommand(String cmd) {
  int angles[4] = {-1, -1, -1, -1};
  int index = 0;
  int start = 0;
  int commaIndex = cmd.indexOf(',');

  while (commaIndex != -1 && index < 3) {
    String part = cmd.substring(start, commaIndex);
    part.trim();
    angles[index] = part.toInt();
    start = commaIndex + 1;
    commaIndex = cmd.indexOf(',', start);
    index++;
  }
  if (index < 4) {
    String part = cmd.substring(start);
    part.trim();
    angles[index] = part.toInt();
  }

  bool valid = true;
  for (int i = 0; i < 4; i++) {
    if (angles[i] < 0 || angles[i] > 180) {
      valid = false;
      break;
    }
  }

  if (valid) {
    for (int i = 0; i < 4; i++) {
      setServoAngle(i, angles[i]);
    }
    Serial.println("Servos set.");
  } else {
    Serial.println("Invalid angles. Please send 4 values 0-180 separated by commas.");
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Initialize starting position (0,0,0,180)
  setServoAngle(SERVO_BASE, 0);
  setServoAngle(SERVO_SHOULDER, 0);
  setServoAngle(SERVO_ELBOW, 0);
  setServoAngle(SERVO_GRIPPER, 180);

  Serial.println("Bluetooth Servo Control Ready");
  Serial.println("Commands:");
  Serial.println(" - RECORD_START");
  Serial.println(" - RECORD_STOP");
  Serial.println(" - PLAY");
  Serial.println(" - Send 4 angles e.g. 90,45,135,180");
  Serial.println(" - IK X,Y,Z        (elbow-up solution)");
  Serial.println(" - IKDOWN X,Y,Z    (elbow-down solution)");
}

// ===== Loop =====
void loop() {
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() > 0) {
      Serial.print("Received: ");
      Serial.println(cmd);

      if (cmd.equalsIgnoreCase("RECORD_START")) {
        startRecording();
      } else if (cmd.equalsIgnoreCase("RECORD_STOP")) {
        stopRecording();
      } else if (cmd.equalsIgnoreCase("PLAY")) {
        startPlayback();
      } else if (cmd.startsWith("IKDOWN ")) {
        String params = cmd.substring(7);
        handleIKCommand(params, false); // elbowDown
      } else if (cmd.startsWith("IK ")) {
        String params = cmd.substring(3);
        handleIKCommand(params, true); // elbowUp
      } else {
        if (!isPlaying) {
          handleAnglesCommand(cmd);
        } else {
          Serial.println("Currently playing back. Send RECORD_STOP to stop playback.");
        }
      }
    }
  }

  if (isRecording) {
    unsigned long now = millis();
    if (now - lastRecordTime >= 4000) {
      if (recordCount < MAX_RECORDS) {
        for (int i = 0; i < 4; i++) {
          recordAngles[recordCount][i] = getServoAngle(i);
        }
        if (recordCount == 0) {
          recordTimes[recordCount] = 0;
        } else {
          recordTimes[recordCount] = now - lastRecordTime;
        }
        recordCount++;
        lastRecordTime = now;
        Serial.print("Recorded step ");
        Serial.println(recordCount);
      } else {
        Serial.println("Record buffer full. Stopping recording.");
        stopRecording();
      }
    }
  }

  if (isPlaying) {
    unsigned long now = millis();
    if (playIndex < recordCount) { // fixed boundary
      if (now - lastPlayStepTime >= recordTimes[playIndex]) {
        for (int i = 0; i < 4; i++) {
          setServoAngle(i, recordAngles[playIndex][i]);
        }
        Serial.print("Playback step ");
        Serial.println(playIndex + 1);
        lastPlayStepTime = now;
        playIndex++;
      }
    } else {
      Serial.println("Playback finished.");
      isPlaying = false;
    }
  }
}
