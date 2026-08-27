#include <Arduino.h>
#include <Bluepad32.h>
#include "controller.h"

static const float DEADZONE = 0.12;
static const float AXIS_MAX = 512.0f;

static ControllerPtr pad = nullptr;
static ControlInput input;

static bool prevCross, prevCircle, prevTriangle, prevSquare; // prev frame state

static float normalize(int32_t raw){
    float v = float(raw) / AXIS_MAX;
    float mag = fabsf(v);
    if (mag < DEADZONE) return 0.0f;
    float sign = (v<0.0f) ?  -1.0f : 1.0f;
    return (sign * constrain((mag - DEADZONE) / (1.0f - DEADZONE), 0.0f, 1.0f));
 }

static void onConnect(ControllerPtr ctl) {
  if (pad != nullptr || !ctl->isGamepad()) return;
  pad = ctl;
  Serial.printf("Controller connected: %s\n", ctl->getModelName().c_str());
}

static void onDisconnect(ControllerPtr ctl) {
  if (pad != ctl) return;
  pad = nullptr;
  Serial.println("Controller disconnected");
}

void ControllerBegin() {
  BP32.setup(&onConnect, &onDisconnect);
  BP32.enableVirtualDevice(false);
  input = ControlInput{};
  prevCross = prevCircle = prevTriangle = prevSquare = false;
}

void ControllerUpdate() {
  BP32.update();

  if (pad == nullptr || !pad->isConnected()) {
    bool wasConnected = input.connected;
    input = ControlInput{};
    prevCross = prevCircle = prevTriangle = prevSquare = false;
    if (wasConnected) Serial.println("Control lost, zeroing input");
    return;
  }

  input.connected = true;

  input.x   = normalize(pad->axisX());
  input.y   = -normalize(pad->axisY());
  input.phi = normalize(pad->axisRX());

  bool cross    = pad->a();
  bool circle   = pad->b();
  bool square   = pad->x();
  bool triangle = pad->y();

  input.btnCrossPress    = cross    && !prevCross;
  input.btnCirclePress   = circle   && !prevCircle;
  input.btnSquarePress   = square   && !prevSquare;
  input.btnTrianglePress = triangle && !prevTriangle;

  prevCross = cross;  prevCircle = circle;
  prevSquare = square;  prevTriangle = triangle;
}

const ControlInput& Control() {
  return input;
}