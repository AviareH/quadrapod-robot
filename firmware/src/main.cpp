#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "kinematics.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 130
#define SERVOMAX 520

const Point staticStance = {58.0f, 43.0f, -58.0f};
const int STEPS = 14;

Point stepList[STEPS] = {
  {58.0f,  43.0f, -58.0f},
  {58.0f,  48.0f, -48.5f},
  {58.0f,  55.5f, -42.4f},
  {58.0f,  63.0f, -38.9f},
  {58.0f,  76.5f, -36.7f},
  {58.0f,  90.0f, -38.9f},
  {58.0f,  97.5f, -42.4f},
  {58.0f, 105.0f, -48.5f},
  {58.0f, 110.0f, -58.0f},
  {58.0f,  97.0f, -58.0f},
  {58.0f,  84.0f, -58.0f},
  {58.0f,  71.0f, -58.0f},
  {58.0f,  57.0f, -58.0f},
  {58.0f,  43.0f, -58.0f}
};

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
Servo BLKnee = {13, 0, 6};
Servo BLHip  = {14, 180, -6};

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
  delay(3000);
  for (int i = 0; i < 4; i++){
    MoveLeg(legs[i],staticStance);
    delay(500);
  }
}

const float X_OUT   = 58.0f;
const float Y_FRONT = 110.0f;
const float Y_BACK  = 43.0f;
const float Z_DOWN  = -58.0f;
const float LIFT    = 21.0f;
const float PUSH    = (Y_FRONT - Y_BACK) / 2.0f;   // 33.5

const int SWING_SUB = 8;
const int PUSH_SUB  = 6;

float legY[4];

void initGait() {
  legY[0] = Y_BACK;          // FL
  legY[1] = Y_BACK + PUSH;   // FR
  legY[2] = Y_BACK + PUSH;   // BR
  legY[3] = Y_FRONT;         // BL
  for (int l = 0; l < 4; l++)
    MoveLeg(legs[l], {X_OUT, legY[l], Z_DOWN});
}

void swingLeg(int l) {
  float y0 = legY[l];
  for (int s = 1; s <= SWING_SUB; s++) {
    float t = (float)s / SWING_SUB;
    Point p;
    p.x = X_OUT;
    p.y = y0 + (Y_FRONT - y0) * t;
    p.z = Z_DOWN + LIFT * sinf(PI * t);
    MoveLeg(legs[l], p);
    delay(75);
  }
  legY[l] = Y_FRONT;
}

void pushAll() {
  float y0[4];
  for (int l = 0; l < 4; l++) y0[l] = legY[l];
  for (int s = 1; s <= PUSH_SUB; s++) {
    float t = (float)s / PUSH_SUB;
    for (int l = 0; l < 4; l++)
      MoveLeg(legs[l], {X_OUT, y0[l] - PUSH * t, Z_DOWN});
    delay(75);
  }
  for (int l = 0; l < 4; l++) legY[l] -= PUSH;
}

void loop() {
  swingLeg(0);   // FL reaches out
  pushAll();     // body advances
  swingLeg(2);   // BR rebalances
  delay(500);

  swingLeg(1);   // FR reaches out
  pushAll();     // body advances
  swingLeg(3);   // BL rebalances
  delay(500);
}