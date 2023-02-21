#include <Arduino.h>
#include "ReflectanceSensor.h"
#include <Romi32U4.h>
#define FARLEFTPIN 20 //Analog 2, sensor 11 (Far Left)
#define LEFTPIN 21 //Analog 3, sensor 7 (Left)
#define RIGHTPIN 22 //Analog 4, sensor 3 (Right)
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
int ReflectanceSensor::readLeft()
{
    return analogRead(LEFTPIN);
}