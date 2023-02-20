#include <Arduino.h>
#include "ReflectanceSensor.h"
#include <Romi32U4.h>

ReflectanceSensor::ReflectanceSensor()
{
}

void ReflectanceSensor::setup()
{
    pinMode(20, INPUT); //Analog 2, sensor 11
    pinMode(21, INPUT); //Analog 3, sensor 9
    pinMode(22, INPUT); //Analog 4, sensor 7
}
int* ReflectanceSensor::readSensor()
{
    readings[0] = analogRead(20);
    readings[1] = analogRead(21);
    readings[2] = analogRead(22);
    return readings; 
}