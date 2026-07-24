#include <Arduino.h>
#include <ESP32Servo.h> 
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 130
#define SERVOMAX 520

int FLFoot=0; //starts at 0, max at 180
int FLKnee=1; //starts at 180, max at 15
int FLHip=2; // starts at 0, max at 150

int FRFoot=4; //starts at 0, max at 180
int FRKnee=5; //starts at 180, max at 15
int FRHip=6; // starts at 180, max at 30

int BRFoot=8; //starts at 0, max at 180
int BRKnee=9; //starts at 180, max at 15
int BRHip=10; // starts at 0, max at 150

int BLFoot=12; //starts at 0, max at 180
int BLKnee=13; //starts at 180, max at 15
int BLHip=14; // starts at 180, max at 30


void setServoAngle(uint8_t channel, float angle) {
  float pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
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
  
  delay(2000); 
}

void loop() {
  setServoAngle(BRFoot, 0);
  delay(1000);
  setServoAngle(BRFoot, 90);
  delay(1000);
  setServoAngle(BRFoot, 180);
  delay(1000);
}