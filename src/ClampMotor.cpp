#include <Arduino.h>
#include "ClampMotor.h"
#include <Romi32U4.h>

ClampMotor::ClampMotor()
{
}

void ClampMotor::setup()
{
    servo.attach();
    pinMode(18, INPUT);
}

long ClampMotor::getPosition()
{
    return analogRead(0);
}

void ClampMotor::reset()
{
    servo.detach();
}

void ClampMotor::setEffortWithDeadband(int effort)
{
    if(effort != 0) {
        effort = constrain(effort, -600, 600);
        effort = effort * ((600.0f - Deadband) / 600.0f);
        if(effort > 0) effort += Deadband;
        else           effort -= Deadband;
    }
    setEffort(effort);
}


void ClampMotor::setEffort(int effort)
{
    if(effort == 0) servo.detach();
    else {
        effort = constrain(effort, -600, 600);
        servo.writeMicroseconds(effort + neutral);
    }
}

bool ClampMotor::moveTo(int target)
{
    target = constrain(target, 0, ADCMAX);
    int pos = getPosition();
    int deltaP = target - pos;
    /*Serial.print(deltaP);
    Serial.print("\t");
    Serial.println(deltaP * kp);*/
    if(abs(deltaP) < tolerance || (deltaP * kp > 0 && pos == 0) || (deltaP * kp < 0 && pos > 1020)) {
        setEffort(0);
        return true;
    }
    setEffortWithDeadband(deltaP * kp);
    return false;
}