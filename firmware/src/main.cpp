#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- PULSE WIDTH CALIBRATION ---
// PCA9685 uses 12-bit resolution (0-4096 ticks) at 50Hz (20ms period)
// 1 tick = 20ms / 4096 = ~4.88 microseconds
// Standard servo pulse: 1ms (0°) to 2ms (180°)
// 1ms = ~205 ticks, 2ms = ~410 ticks
// Start conservative — widen carefully during calibration
#define SERVOMIN 205
#define SERVOMAX 410

// --- SERVO CHANNEL MAP ---
// Address : Name         Notes
//   0     : FL Foot      
//   1     : FL Knee      
//   2     : FL Hip       
//   4     : FR Foot      
//   5     : FR Knee      
//   6     : FR Hip       
//   8     : BR Foot      
//   9     : BR Knee      
//  10     : BR Hip       
//  12     : BL Foot      
//  13     : BL Knee      
//  14     : BL Hip       

const int NUM_SERVOS = 12;

struct ServoInfo {
  uint8_t channel;
  const char* name;
};

ServoInfo servos[NUM_SERVOS] = {
  {0,  "FL Foot"},
  {1,  "FL Knee"},
  {2,  "FL Hip"},
  {4,  "FR Foot"},
  {5,  "FR Knee"},
  {6,  "FR Hip"},
  {8,  "BR Foot"},
  {9,  "BR Knee"},
  {10, "BR Hip"},
  {12, "BL Foot"},
  {13, "BL Knee"},
  {14, "BL Hip"}
};

// Maps address index (0-11) to descriptor above
// Address 0  = index 0  (FL Foot)
// Address 1  = index 1  (FL Knee)
// etc.

void setServoAngle(uint8_t channel, float angle) {
  float pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, (int)pulse);
}

void printServoList() {
  Serial.println(F("\n--- SERVO LIST ---"));
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(servos[i].name);
    Serial.print(F("  (PCA channel "));
    Serial.print(servos[i].channel);
    Serial.println(F(")"));
  }
  Serial.println(F("------------------"));
}

String readSerialLine() {
  String input = "";
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (input.length() > 0) return input;
      } else {
        input += c;
        Serial.print(c); // echo back
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  Serial.println(F("\n=== SERVO CALIBRATION TOOL ==="));
  Serial.println(F("Use this tool to find zero positions and physical limits for each servo."));
  Serial.println(F("SERVOMIN/SERVOMAX are conservative — widen in code once limits are confirmed."));
  Serial.println(F("Tip: listen for grinding. If you hear it, the servo has hit a hard stop. Back off immediately."));
}

void loop() {
  printServoList();
  Serial.print(F("\nEnter servo index (0-11): "));

  String indexInput = readSerialLine();
  Serial.println();

  int idx = indexInput.toInt();
  if (idx < 0 || idx >= NUM_SERVOS) {
    Serial.println(F("Invalid index. Try again."));
    return;
  }

  uint8_t channel = servos[idx].channel;
  Serial.print(F("Selected: "));
  Serial.print(servos[idx].name);
  Serial.print(F(" on PCA channel "));
  Serial.println(channel);
  Serial.println(F("Enter angle (0-180) or Q to change servo:"));

  while (true) {
    Serial.print(F("  Angle > "));
    String angleInput = readSerialLine();
    Serial.println();

    if (angleInput.equalsIgnoreCase("Q")) {
      Serial.println(F("Changing servo...\n"));
      break;
    }

    int angle = angleInput.toInt();

    if (angle < 0 || angle > 180) {
      Serial.println(F("  Out of range. Enter 0-180."));
      continue;
    }

    setServoAngle(channel, angle);
    Serial.print(F("  >> Set "));
    Serial.print(servos[idx].name);
    Serial.print(F(" to "));
    Serial.print(angle);
    Serial.println(F("°"));
  }
}