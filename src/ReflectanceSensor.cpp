#include <Arduino.h>
#include "ReflectanceSensor.h"
#include <Romi32U4.h>
#define FARLEFTPIN 22 //Analog 2, sensor 11 (Far Right)
#define LEFTPIN 21 //Analog 3, sensor 7 (Right)
#define RIGHTPIN 20 //Analog 4, sensor 3 (Left)
ReflectanceSensor::ReflectanceSensor()
{
}

/**
 * @brief Set up the microcontroller to be able to read the data from the sensor
 * 
 */
void ReflectanceSensor::setup()
{
    pinMode(FARLEFTPIN, INPUT);
    pinMode(LEFTPIN, INPUT); 
    pinMode(RIGHTPIN, INPUT); 
}
/**
 * @brief Reads the value of the right reflectance sensor
 * 
 * @return int the value read
 */
int ReflectanceSensor::readRight()
{
    return analogRead(RIGHTPIN);
}
/**
 * @brief Reads the value of the far left reflectance sensor
 * 
 * @return int the value read
 */
int ReflectanceSensor::readFarLeft()
{
    return analogRead(FARLEFTPIN);
}
/**
 * @brief Determines if the far left reflectance sensor is over a line
 * 
 * @return true if it is over a black line
 * @return false if it isn't over a black line
 */
bool ReflectanceSensor::farLeftOverLine()
{
    return readFarLeft() > 600;
}
/**
 * @brief Reads the value of the left reflectance sensor
 * 
 * @return int the value read
 */
int ReflectanceSensor::readLeft()
{
    return analogRead(LEFTPIN);
}