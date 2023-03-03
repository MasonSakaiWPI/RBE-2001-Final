#include <Arduino.h>
#include <BlueMotor.h>
#include <Romi32U4.h>

long oldValue = 0;
long newValue;
long count = 0;
unsigned time = 0;

BlueMotor::BlueMotor()
{
}

/**
 * @brief Sets up the microcontroller to control the bluemotor (associated h bridge output waveforms and quadrature encoder inputs)
 */
void BlueMotor::setup()
{
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    TCCR1A = 0xA8; //0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; //0b00010001;
    ICR1 = 400;
    OCR1C = 0;

    attachInterrupt(digitalPinToInterrupt(ENCA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB), isrB, CHANGE);
    reset();
}
/**
 * @brief Gets the position of the blue motor
 * 
 * @return The encoder position as a long
 */
long BlueMotor::getPosition()
{
    long tempCount = 0;
    noInterrupts();
    tempCount = count;
    interrupts();
    return tempCount;
}

/**
 * @brief Resets the position counter to zero
 */
void BlueMotor::reset()
{
    noInterrupts();
    count = 0;
    interrupts();
}

/**
 * @brief The ISR for sensor A
 */
void BlueMotor::isrA()
{
    if(digitalRead(1) == digitalRead(0))
        count--;
    else count++;
}
/**
 * @brief The ISR for sensor B
 */
void BlueMotor::isrB()
{
    if(digitalRead(1) == digitalRead(0))
        count++;
    else count--;
}

/**
 * @brief Applies the effort given with the set deadband, range -400 to 400
 * 
 * @return The applied effort with deadband
 */
int BlueMotor::setEffortWithDeadband(int effort)
{
    if (effort < 0) return -setEffortWithDeadband(-effort, true);
    else            return setEffortWithDeadband(effort, false);
}
/**
 * @brief Applies the effort given with a deadband, range -400 to 400
 * @brief This version takes unsigned, and a "negative" value would be clockwise
 * 
 * @return The applied effort with deadband
 */
int BlueMotor::setEffortWithDeadband(int effort, bool clockwise)
{
    if(effort != 0) {
        effort = constrain(effort, 0, 400);
        effort = effort * DBMult + Deadband;
    }
    setEffort(effort, clockwise);
    return effort;
}

/**
 * @brief Sets the effort of the motor, range -400 to 400
 */
void BlueMotor::setEffort(int effort)
{
    if (effort < 0) setEffort(-effort, true);
    else            setEffort(effort, false);
}
/**
 * @brief Sets the effort of the motor, range -400 to 400.
 * @brief This version takes unsigned, and a "negative" value would be clockwise
 */
void BlueMotor::setEffort(int effort, bool clockwise)
{
    if (clockwise)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    OCR1C = constrain(effort, 0, 390);
}

/**
 * @brief Stops the motor if the position of it ends up outside of normal operating range
 * 
 */
void BlueMotor::safetyCheck() {
    long pos = getPosition();
    if(pos < -50) setEffort(0);
    else if (pos > 3700) setEffort(0);
}

/**
 * @brief Moves the motor to the given position. This uses the motor deadband
 * 
 * @return true if the motor has reached it's target position,
 * @return false otherwise
 */
bool BlueMotor::moveTo(long target)
{
    long deltaP = target - count;
    if(abs(deltaP) < tolerance) {
        setEffort(0);
        return true;
    }
    if(deltaP * kp >= INT16_MAX)       deltaP = INT16_MAX / kp;
    else if (deltaP * kp <= INT16_MIN) deltaP = INT16_MIN / kp;
    setEffortWithDeadband(deltaP * kp);
    return false;
}
