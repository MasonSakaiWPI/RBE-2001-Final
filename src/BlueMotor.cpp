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
 * 
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

long BlueMotor::getPosition()
{
    long tempCount = 0;
    noInterrupts();
    tempCount = count;
    interrupts();
    return tempCount;
}

void BlueMotor::reset()
{
    noInterrupts();
    count = 0;
    interrupts();
}


void BlueMotor::isrA()
{
    if(digitalRead(1) == digitalRead(0))
        count--;
    else count++;
}

void BlueMotor::isrB()
{
    if(digitalRead(1) == digitalRead(0))
        count++;
    else count--;
}

int BlueMotor::setEffortWithDeadband(int effort)
{
    if (effort < 0) return setEffortWithDeadband(-effort, true);
    else            return setEffortWithDeadband(effort, false);
}

int BlueMotor::setEffortWithDeadband(int effort, bool clockwise)
{
    if(effort != 0)
        effort = effort * ((400.0f - Deadband) / 400.0f) + Deadband;
    setEffort(effort, clockwise);
    return effort;
}


void BlueMotor::setEffort(int effort)
{
    if (effort < 0) setEffort(-effort, true);
    else            setEffort(effort, false);
}

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
    OCR1C = constrain(effort, 0, 400);
}

bool BlueMotor::moveTo(long target)
{
    long deltaP = target - count;
    if(abs(deltaP) < tolerance) {
        setEffort(0);
        return true;
    }
    /*Serial.print(deltaP);
    Serial.print("\t");
    Serial.println(deltaP * kp);*/
    if(deltaP * kp >= INT16_MAX)       deltaP = INT16_MAX / kp;
    else if (deltaP * kp <= INT16_MIN) deltaP = INT16_MIN / kp;
    setEffortWithDeadband(deltaP * kp);
    return false;
}
