#pragma once
#include <Servo32u4.h>

const int ADCMAX = 1023;

class ClampMotor
{
public:
    ClampMotor();
    void setEffort(int effort);
    void setEffortWithDeadband(int effort);
    bool moveTo(int position);
    long getPosition();
    void setup();
    void reset();

private:
    const int tolerance = 5;
    const int kp = -10;
    const int Deadband = 0;
    const int neutral = 1500;
    Servo32U4Pin5 servo;
};