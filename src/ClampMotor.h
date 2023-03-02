#pragma once
#include <Servo32u4.h>

const int ADCMAX = 1023;

class ClampMotor
{
public:
    ClampMotor();
    bool moveTo(int position);
    bool grabbedPlate();
    int getPosition();
    void setup();
    void reset();
    bool moveToPot(int position);

private:
    const int neutral = 1500,
              tolerance = 10,
              grabbedPosition = 280;
    Servo32U4Pin5 servo;
};