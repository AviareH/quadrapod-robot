#pragma once
#include <stdint.h>

struct ControlInput{
    bool connected;

    float x;
    float y;
    float phi;

    bool btnCrossPress, btnCirclePress, btnTrianglePress, btnSquarePress;
};

void ControllerBegin();
void ControllerUpdate();
const ControlInput& Control();