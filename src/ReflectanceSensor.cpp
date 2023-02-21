#include <Arduino.h>
#include "ReflectanceSensor.h"
#include <Romi32U4.h>
#define LEFTPIN 21 //Analog 3, sensor 9 (Left)
#define CENTERPIN 20 //Analog 2, sensor 11 (Center)
#define RIGHTPIN 22 //Analog 4, sensor 7 (Right)
ReflectanceSensor::ReflectanceSensor()
{
}

void ReflectanceSensor::setup()
{
    pinMode(CENTERPIN, INPUT);
    pinMode(LEFTPIN, INPUT); 
    pinMode(RIGHTPIN, INPUT); 
}
int ReflectanceSensor::readRight()
{
    return analogRead(RIGHTPIN);
}
int ReflectanceSensor::readCenter()
{
    return analogRead(CENTERPIN);
}
int ReflectanceSensor::readLeft()
{
    return analogRead(LEFTPIN);
}