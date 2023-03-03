#include <Arduino.h>
#include "ClampMotor.h"
#include <Romi32U4.h>

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
}

bool ClampMotor::grabbedPlate()
{
    return getPosition() < grabbedPosition + tolerance;
}


/**
 * @brief Moves the motor to the given position. This uses the motor deadband
 * 
 * @return 0 if the motor is in motion,
 * @return 1 if the motor has reached it's final position or is trying to move out of bounds,
 * @return 2 if the motor has been stationary for too long (requres immediate handling)
 */
bool ClampMotor::moveTo(int target)
{
    target = constrain(target, -100, 250);
    servo.writeMicroseconds(neutral + target);
    return grabbedPlate();
}
/*
clampPos = -150; //262
clampPos = 0; //277 stuck
clampPos = 250; //335
*/