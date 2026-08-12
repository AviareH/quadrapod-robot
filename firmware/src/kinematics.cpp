#include <Arduino.h>
#include <math.h>
#include "kinematics.h"

const float KneeLink = 61.5;
const float FootLink = 75.5;
const float HipOffset = 31.5;
const float reachMax = KneeLink + FootLink;
const float reachMin = fabs(KneeLink - FootLink);

JointAngles SolveIK(const Point& target, bool mirrored){
  JointAngles angles;

  angles.valid = true;
  angles.theta1 = angles.theta2 = angles.theta3 = 0.0f;

  float horizontalDist = sqrt(target.x*target.x + target.y*target.y);
  float planarReach = horizontalDist - HipOffset;

  if (planarReach < 0.0f) {
    Serial.printf("UNREACHABLE: d=%.1f is inside HipOffset\n", horizontalDist);
    angles.valid = false;
    return angles;
  }

  float LegSpan = sqrt(planarReach*planarReach + target.z*target.z);

  if (LegSpan > reachMax || LegSpan < reachMin) {
    Serial.printf("UNREACHABLE: c=%.1f outside [%.1f, %.1f]\n", LegSpan, reachMin, reachMax);
    angles.valid = false;
    return angles;
  }

  float rawTheta1 = atan2(target.x, target.y) * 180.0f / M_PI;
  float rawTheta2 = (atan2(planarReach, -target.z) + acos(constrain((KneeLink*KneeLink + LegSpan*LegSpan - FootLink*FootLink) / (2.0f*KneeLink*LegSpan), -1.0f, 1.0f))) * 180.0f / M_PI;
  float rawTheta3 = acos(constrain((KneeLink*KneeLink + FootLink*FootLink - LegSpan*LegSpan) / (2.0f*KneeLink*FootLink), -1.0f, 1.0f)) * 180.0f / M_PI;

  if (mirrored) {
    angles.theta1 = 180.0f - rawTheta1;
    angles.theta2 = 180.0f - rawTheta2;
    angles.theta3 = rawTheta3;
  } else {
    angles.theta1 = rawTheta1;
    angles.theta2 = rawTheta2;
    angles.theta3 = 180.0f - rawTheta3;
  }

  if (angles.theta1 < 0.0f || angles.theta1 > 180.0f ||
      angles.theta2 < 0.0f || angles.theta2 > 180.0f ||
      angles.theta3 < 0.0f || angles.theta3 > 180.0f) {
    Serial.printf("OUT OF SERVO RANGE: hip %.1f  knee %.1f  foot %.1f\n", angles.theta1, angles.theta2, angles.theta3);
    angles.valid = false;
    return angles;
  }

  return angles;
}