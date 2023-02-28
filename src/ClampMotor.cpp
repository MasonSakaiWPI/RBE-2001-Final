#include <Arduino.h>
#include "ClampMotor.h"
#include <Romi32U4.h>

unsigned const int LUTTimeout = 1000,
          LUTThreshold = 8;
int lastUpdatedPosition = 0,
    lastTarget = -1;
unsigned long lastUpdatedTime = 0;

ClampMotor::ClampMotor()
{
}

/**
 * @brief Sets up the motor control
 */
void ClampMotor::setup()
{
    servo.attach();
    pinMode(18, INPUT);
}

/**
 * @brief Gets the position of the linear potentiomenter
 * 
 * @return The position as a int
 */
int ClampMotor::getPosition()
{
    return analogRead(0);
}

/**
 * @brief Disconnects the motor (Setup needs to be rerun)
 */
void ClampMotor::reset()
{
    servo.detach();
    lastUpdatedPosition = getPosition();
    lastUpdatedTime = millis();
}

/**
 * @brief Applies the effort given with the set deadband, range -600 to 600
 */
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
/**
 * @brief Sets the effort of the motor, range -600 to 600
 */
void ClampMotor::setEffort(int effort)
{
    if(effort == 0) servo.detach();
    else {
        effort = constrain(effort, -600, 600);
        servo.writeMicroseconds(effort + neutral);
    }
}

/**
 * @brief Moves the motor to the given position. This uses the motor deadband
 * 
 * @return 0 if the motor is in motion,
 * @return 1 if the motor has reached it's final position or is trying to move out of bounds,
 * @return 2 if the motor has been stationary for too long (requres immediate handling)
 */
byte ClampMotor::moveTo(int target)
{
    if(target != lastTarget) {
        lastTarget = target;
        lastUpdatedPosition = getPosition();
        lastUpdatedTime = millis();
    }

    target = constrain(target, 0, ADCMAX);
    int pos = getPosition();
    int deltaP = target - pos;
    /*Serial.print(deltaP);
    Serial.print("\t");
    Serial.println(deltaP * kp);*/
    if(abs(deltaP) < tolerance || (deltaP * kp > 0 && pos == 0) || (deltaP * kp < 0 && pos > 1020)) {
        setEffort(0);
        return 1;
    } else if(abs(lastUpdatedPosition - pos) > LUTThreshold) {
        lastUpdatedPosition = pos;
        if(deltaP > 0) lastUpdatedPosition -= LUTThreshold / 2;
        else lastUpdatedPosition += LUTThreshold / 2;
        lastUpdatedTime = millis();
        /*Serial.print(lastUpdatedPosition);
        Serial.print("\t");
        Serial.print(target);
        Serial.print("\t");
        Serial.println(pos);*/
    }
    else if(lastUpdatedTime + LUTTimeout < millis()) {
        //Serial.print("Hit at ");
        //Serial.println(pos);
        return 2;
    }
    setEffortWithDeadband(deltaP * kp);
    return 0;
}