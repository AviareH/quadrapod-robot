#include <Arduino.h>
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

JointAngles SolveIK(const Point& target, bool mirrored){
  JointAngles angles;

  float d = sqrt(target.x*target.x+target.y*target.y);
  float r = d-HipOffset;
  float c = sqrt(r*r + target.z*target.z);

  float rawTheta1 = atan2(target.x, target.y) * 180.0f / M_PI;
  float rawTheta2 = (atan2(r, -target.z) + acos((KneeLink*KneeLink + c*c - FootLink*FootLink) / (2.0f*KneeLink*c))) * 180.0f / M_PI;
  float rawTheta3 = acos((KneeLink*KneeLink + FootLink*FootLink - c*c) / (2.0f*KneeLink*FootLink)) * 180.0f / M_PI;

  if (mirrored) {
    angles.theta1 = 180.0f - rawTheta1;
    angles.theta2 = 180.0f - rawTheta2;
    angles.theta3 = rawTheta3;
  } else {
    angles.theta1 = rawTheta1;
    angles.theta2 = rawTheta2;
    angles.theta3 = 180.0f - rawTheta3;
  }

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

// --- SERIAL INPUT HELPER ---
// Blocks until a full line is received, echoing characters back as typed.
String readSerialLine() {
  String input = "";
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (input.length() > 0) return input;
      } else {
        input += c;
        Serial.print(c);
      }
    }
  }
}

// Finds a leg by name (case-insensitive). Returns nullptr if not found.
Leg* findLeg(const String& name) {
  for (int i = 0; i < 4; i++) {
    if (name.equalsIgnoreCase(legs[i].name)) {
      return &legs[i];
    }
  }
  return nullptr;
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50); 
  delay(10);

  Serial.println(F("\n=== LEG IK TEST TOOL ==="));
  Serial.println(F("Enter a leg (FL, FR, BR, BL) and a target X Y Z to solve IK and move that leg."));
}

void loop() {
  Serial.print(F("\nLeg (FL/FR/BR/BL): "));
  String legInput = readSerialLine();
  Serial.println();

  Leg* leg = findLeg(legInput);
  if (leg == nullptr) {
    Serial.println(F("Unknown leg. Try again."));
    return;
  }

  Serial.print(F("Target X Y Z (e.g. 100 100 100): "));
  String pointInput = readSerialLine();
  Serial.println();

  Point target;
  int parsed = sscanf(pointInput.c_str(), "%f %f %f", &target.x, &target.y, &target.z);

  if (parsed != 3) {
    Serial.println(F("Could not parse three numbers. Try again."));
    return;
  }

  JointAngles angles = SolveIK(target, leg->mirrored);

  Serial.printf("[%s] target (%.1f, %.1f, %.1f) -> hip %.1f  knee %.1f  foot %.1f\n",
    leg->name, target.x, target.y, target.z,
    angles.theta1, angles.theta2, angles.theta3);

  
  setServoAngle(*leg->hip, angles.theta1);
  setServoAngle(*leg->knee, angles.theta2);
  setServoAngle(*leg->foot, angles.theta3);
} 