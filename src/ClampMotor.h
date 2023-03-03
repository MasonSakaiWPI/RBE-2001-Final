#pragma once
#include <Servo32u4.h>

class ClampMotor
{
public:
    ClampMotor();
    bool moveTo(int position);
    bool grabbedPlate();
    int getPosition();
    void setup();
    void reset();

private:
    const int neutral = 1500,
              tolerance = 10,
              grabbedPosition = 277;
    Servo32U4Pin5 servo;
};

/*
clampPos = -150; //262
clampPos = 0; //277 stuck
clampPos = 250; //335
*/