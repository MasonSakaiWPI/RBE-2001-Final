#include <Arduino.h>
#include "ReflectanceSensor.h"
#include <Romi32U4.h>
#define FARRIGHTPIN 22 //Analog 2, sensor 11 (Far Right)
#define RIGHTPIN 21 //Analog 3, sensor 7 (Right)
#define LEFTPIN 20 //Analog 4, sensor 3 (Left)
ReflectanceSensor::ReflectanceSensor()
{
}

/**
 * @brief Set up the microcontroller to be able to read the data from the sensor
 * 
 */
void ReflectanceSensor::setup()
{
  pinMode(FARRIGHTPIN, INPUT);
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
 * @brief Reads the value of the far right reflectance sensor
 * 
 * @return int the value read
 */
int ReflectanceSensor::readFarRight()
{
  return analogRead(FARRIGHTPIN);
}
/**
 * @brief Determines if the far right reflectance sensor is over a line
 * 
 * @return true if it is over a black line
 * @return false if it isn't over a black line
 */
bool ReflectanceSensor::farRightOverLine()
{
  return readFarRight() > LineThreshold;
}
/**
 * @brief Determines if the far right reflectance sensor is over a line
 * 
 * @return true if it is over a black line
 * @return false if it isn't over a black line
 */
bool ReflectanceSensor::rightOverLine()
{
  return readRight() > LineThreshold;
}
/**
 * @brief Determines if the far right reflectance sensor is over a line
 * 
 * @return true if it is over a black line
 * @return false if it isn't over a black line
 */
bool ReflectanceSensor::leftOverLine()
{
  return readLeft() > LineThreshold;
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

/**
 * @brief Determines the state of the left line sensor, includes transitioning states
 * 
 * @return The state of the left line sensor, see getLeftLineState for state values
 */
byte ReflectanceSensor::updateLeftLineState() {
  if(leftOverLine()) {
    if(leftState == LOW || leftState == FALLING) return (leftState = RISING);
    else return (leftState = HIGH);
  } else {
    if(leftState == LOW || leftState == FALLING) return (leftState = LOW);
    else return (leftState = FALLING);
  }
}
/**
 * @brief Determines the state of the right line sensor, includes transitioning states
 * 
 * @return The state of the left line sensor, see getRightLineState for state values
 */
byte ReflectanceSensor::updateRightLineState() {
  if(rightOverLine()) {
    if(rightState == LOW || rightState == FALLING) return (rightState = RISING);
    else return (rightState = HIGH);
  } else {
    if(rightState == LOW || rightState == FALLING) return (rightState = LOW);
    else return (rightState = FALLING);
  }
}
/**
 * @brief Gets the state of the left line sensor, includes transitioning states
 * 
 * @returns LOW (0) when not over the line,
 * @returns HIGH (1) when over the line,
 * @returns FALLING (2) when LOW and last state was HIGH or RISING, and
 * @returns RISING (3) when HIGH and last state was LOW or FALLING
 */
byte ReflectanceSensor::getLeftLineState() {
    return leftState;
}
/**
 * @brief Gets the state of the right line sensor, includes transitioning states
 * 
 * @returns LOW (0) when not over the line,
 * @returns HIGH (1) when over the line,
 * @returns FALLING (2) when LOW and last state was HIGH or RISING, and
 * @returns RISING (3) when HIGH and last state was LOW or FALLING
 */
byte ReflectanceSensor::getRightLineState() {
    return rightState;
}