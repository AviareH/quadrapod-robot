#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "kinematics.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 130
#define SERVOMAX 520

const float X_OUT     = 58.0f;
const float Y_FRONT   = 120.0f;
const float Y_BACK    = 43.0f;
const float Z_DOWN    = -58.0f;
const float LIFT      = 21.0f;
const float STEP_BACK = (Y_FRONT - Y_BACK) / 3.0f;

const int SUBSTEPS = 9;
const int ORDER[4] = {0, 2, 1, 3};   // FL, BR, FR, BL

float legY[4];

struct Servo {
  uint8_t channel;
  float angle;
  int offset;
};

Servo FLFoot = {0, 0, 10};
Servo FLKnee = {1, 180, -15};
Servo FLHip  = {2, 0, 4};

Servo FRFoot = {4, 180, 0};
Servo FRKnee = {5, 0, 10};
Servo FRHip  = {6, 180, -4};

Servo BRFoot = {8, 0, 0};
Servo BRKnee = {9, 180, -9};
Servo BRHip  = {10, 0, 5};

Servo BLFoot = {12, 180, -1};
Servo BLKnee = {13, 0, 12};
Servo BLHip  = {14, 180, -10};

struct Leg {
  const char* name;
  Servo* hip;
  Servo* knee; 
  Servo* foot;
  bool mirrored;
  bool rear;
};

Leg legs[4] = {
  {"FL", &FLHip, &FLKnee, &FLFoot, false, false},
  {"FR", &FRHip, &FRKnee, &FRFoot, true, false},
  {"BR", &BRHip, &BRKnee, &BRFoot, false, true},
  {"BL", &BLHip, &BLKnee, &BLFoot, true, true},
};

void setServoAngle(Servo &servo, float angle) {
  servo.angle = constrain(angle+servo.offset, 0.0f, 180.0f);
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
  if (leg.rear) local.y = 2.0f * 43.0f - local.y;

  JointAngles angles = SolveIK(local, leg.mirrored);

  if (!angles.valid) {
    Serial.printf("[%s] rejected, holding position\n", leg.name);
    return false;
  }

  setServoAngle(*leg.hip,  angles.theta1);
  setServoAngle(*leg.knee, angles.theta2);
  setServoAngle(*leg.foot, angles.theta3);
  return true;
}

void initGait() {
  for (int i = 0; i < 4; i++)
    legY[ORDER[i]] = Y_BACK + i * STEP_BACK;
  for (int l = 0; l < 4; l++) {
    MoveLeg(legs[l], {X_OUT, legY[l], Z_DOWN});
    delay(300);
  }
}

void setup() {
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  initGait();
  delay(1000);
}

void loop() {
  for (int p = 0; p < 4; p++) {
    int swing = ORDER[p];

    for (int s = 1; s <= SUBSTEPS; s++) {
      float t = (float)s / SUBSTEPS;

      for (int l = 0; l < 4; l++) {
        Point tgt;
        tgt.x = X_OUT;
        if (l == swing) {
          tgt.y = Y_BACK + (Y_FRONT - Y_BACK) * t;
          tgt.z = Z_DOWN + LIFT * sinf(PI * t);
        } else {
          tgt.y = legY[l] - STEP_BACK * t;
          tgt.z = Z_DOWN;
        }
        MoveLeg(legs[l], tgt);
      }
      delay(40);
    }

    for (int l = 0; l < 4; l++)
      legY[l] = (l == swing) ? Y_FRONT : legY[l] - STEP_BACK;
  }
}