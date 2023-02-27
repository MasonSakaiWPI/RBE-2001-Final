#include <Arduino.h>
#include "ReflectanceSensor.h"
#include <Romi32U4.h>
#define FARLEFTPIN 22 //Analog 2, sensor 11 (Far Right)
#define LEFTPIN 21 //Analog 3, sensor 7 (Right)
#define RIGHTPIN 20 //Analog 4, sensor 3 (Left)
ReflectanceSensor::ReflectanceSensor()
{
}

void ReflectanceSensor::setup()
{
    pinMode(FARLEFTPIN, INPUT);
    pinMode(LEFTPIN, INPUT); 
    pinMode(RIGHTPIN, INPUT); 
}
int ReflectanceSensor::readRight()
{
    return analogRead(RIGHTPIN);
}
int ReflectanceSensor::readFarLeft()
{
    return analogRead(FARLEFTPIN);
}
bool ReflectanceSensor::farLeftOverLine()
{
    return readFarLeft() > 600;
}
int ReflectanceSensor::readLeft()
{
    return analogRead(LEFTPIN);
}