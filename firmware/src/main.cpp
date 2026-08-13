#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "kinematics.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 130
#define SERVOMAX 520

struct Servo {
  uint8_t channel;
  float angle;
};

Servo FLFoot = {0, 0};
Servo FLKnee = {1, 180};
Servo FLHip  = {2, 0};

Servo FRFoot = {4, 0};
Servo FRKnee = {5, 180};
Servo FRHip  = {6, 180};

Servo BRFoot = {8, 0};
Servo BRKnee = {9, 180};
Servo BRHip  = {10, 0};

Servo BLFoot = {12, 0};
Servo BLKnee = {13, 180};
Servo BLHip  = {14, 180};

// --- LEG SELECTOR TABLE ---
// Groups each leg's three servos with its mirrored flag, so the
// interactive loop knows which physical servos to drive for a given target.
struct Leg {
  const char* name;
  Servo* hip;
  Servo* knee; 
  Servo* foot;
  bool mirrored;
};

Leg legs[4] = {
  {"FL", &FLHip, &FLKnee, &FLFoot, false},
  {"FR", &FRHip, &FRKnee, &FRFoot, true},
  {"BR", &BRHip, &BRKnee, &BRFoot, false},
  {"BL", &BLHip, &BLKnee, &BLFoot, true},
};

void setServoAngle(Servo &servo, float angle) {
  servo.angle = constrain(angle, 0.0f, 180.0f);
  uint16_t pulse = map((int)round(servo.angle), 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo.channel, 0, pulse);
}

void neutral(){
  setServoAngle(FLFoot, 0);
  setServoAngle(FLKnee, 180);
  setServoAngle(FLHip, 0);

  setServoAngle(FRFoot, 180);
  setServoAngle(FRKnee, 0);
  setServoAngle(FRHip, 180);

  setServoAngle(BRFoot, 0);
  setServoAngle(BRKnee, 180);
  setServoAngle(BRHip, 0);

  setServoAngle(BLFoot, 180);
  setServoAngle(BLKnee, 0);
  setServoAngle(BLHip, 180);
}

bool MoveLeg(Leg& leg, const Point& bodyTarget) {
  Point local = bodyTarget;

  JointAngles angles = SolveIK(local, leg.mirrored);

  if (!angles.valid) {
    Serial.printf("[%s] rejected, holding position\n", leg.name);
    return false;
  }

  Serial.printf("[%s] target (%.1f, %.1f, %.1f) -> hip %.1f  knee %.1f  foot %.1f\n",
    leg.name, bodyTarget.x, bodyTarget.y, bodyTarget.z,
    angles.theta1, angles.theta2, angles.theta3);

  setServoAngle(*leg.hip,  angles.theta1);
  setServoAngle(*leg.knee, angles.theta2);
  setServoAngle(*leg.foot, angles.theta3);
  return true;
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50); 
  delay(10);
  neutral();
}

void loop() {
  Point stance = {57.0f, 43.0f, -58.0f};

  // for (int i = 0; i < 4; i++){
  //   MoveLeg(legs[i],stance);
  //   delay(500);
  // }

  // delay(3000);
} 