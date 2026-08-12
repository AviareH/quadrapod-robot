#pragma once

struct Point {
  float x;
  float y;
  float z;
};

struct JointAngles {
  float theta1; // Hip
  float theta2; // Knee
  float theta3; // Foot
  bool valid;   // Reachable?
};

extern const float KneeLink;
extern const float FootLink;
extern const float HipOffset;
extern const float reachMax;
extern const float reachMin;

JointAngles SolveIK(const Point& target, bool mirrored);