#include <Arduino.h>
#include <ESP32Servo.h> 
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 130
#define SERVOMAX 520

const float KneeLink = 61.5;
const float FootLink = 75.5;
const float HipOffset = 31.5;

struct Servo {
  uint8_t channel;
  float angle;
};

struct Point {
  float x;
  float y;
  float z;
};

struct JointAngles {
  float theta1; // Hip
  float theta2; // Knee
  float theta3; // Foot
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


void setServoAngle(Servo &servo, float angle) {
  servo.angle = constrain(angle, 0.0f, 180.0f);
  uint16_t pulse = map((int)round(servo.angle), 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo.channel, 0, pulse);
}

JointAngles SolveIK(const Point& target){
  JointAngles angles;

  float d = sqrt(pow(target.x,2)+pow(target.y,2)) - HipOffset;
  float c = sqrt(pow(d,2)+pow(target.z,2));

  angles.theta1 = (atan2(target.x,target.y))*180/M_PI;
  angles.theta2 = (atan2(d,target.z)+acos((pow(KneeLink,2)+pow(c,2)-pow(FootLink,2))/(2.0*KneeLink*c)))*180/M_PI;
  angles.theta3 = (acos((pow(KneeLink,2)-pow(c,2)+pow(FootLink,2))/(2.0*KneeLink*FootLink)))*180/M_PI;

  return angles;
}


void neutral(){
  setServoAngle(FLFoot, 0);
  setServoAngle(FLKnee, 180);
  setServoAngle(FLHip, 0);
  setServoAngle(FRFoot, 0);
  setServoAngle(FRKnee, 0);
  setServoAngle(FRHip, 180);
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50); 
  delay(10);
  
}

void loop() {
  setServoAngle(BRFoot, 0);
  delay(1000);
  setServoAngle(BRFoot, 90);
  delay(1000);
  setServoAngle(BRFoot, 180);
  delay(1000);
}