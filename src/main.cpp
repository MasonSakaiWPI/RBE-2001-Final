#include <Arduino.h>
#include <Romi32U4.h>
#include <wpi-32u4-lib.h>

#include <Chassis.h>

#include <IRdecoder.h>
#include <ir_codes.h>

#include "Ultrasonic.h"
#include "ReflectanceSensor.h"

#include "BlueMotor.h"
#include "ClampMotor.h"

Romi32U4ButtonB buttonB;

IRDecoder decoder(14);

Ultrasonic sonar;
ReflectanceSensor reflectanceSensor;

BlueMotor blueMotor;
ClampMotor clampMotor;
Chassis chassis;

enum robotStates {Idle, ApproachingRoof, ApproachingStagingArea, PlacingRoof, RemovingRoof, PlacingStagingArea, RemovingStagingArea};
int robotState;
int roofState = 45; //this stores which side of the field the robot is on based on the roof angle

int clampPos = 0, //Clamp target position
    blueMotorPos = 0; //Blue Motor target position
float bme = 0; //Blue motor manual effort

bool approachRoofTest = false,
     blueMotorPosMode = false; //Use BME (false) or blueMotorPos(true)

const long target45 = 1870,  //45d position
           target45_2 = 2350, // raises plate
           target25 = 2600,  //25d position
           target25_2 = 2750, // raises plate
           targetSonar = 1250; //min for clear of sonar
const float sonarPickup = 6.5,
            sonar45 = 9.9,
            sonar25 = 8.1;



bool checkRemote() {
  switch (decoder.getKeyCode())
  { //Everything here is for testing purposes
  case PLAY_PAUSE:
    approachRoofTest = !approachRoofTest;
    break;
  case ENTER_SAVE:
    bme = 0;
    break;
  case UP_ARROW:
    bme += 50;
    break;
  case DOWN_ARROW:
    bme -= 50;
    break;
    
  case SETUP_BTN:
    blueMotor.reset();
    break;
  case STOP_MODE:
    Serial.print("Blue Motor Encoder: ");
    Serial.println(blueMotor.getPosition());
    Serial.print("Clamp Linear Pot: ");
    Serial.println(clampMotor.getPosition());
    Serial.print("Clamp Has Plate: ");
    Serial.println(clampMotor.grabbedPlate());
    Serial.print("Sonar Dist: ");
    Serial.println(sonar.getDistance());
    Serial.print("Line: ");
    Serial.println(analogRead(3));
    Serial.print("Battery Voltage: ");
    Serial.println(readBatteryMillivolts());
    break;

  case NUM_1:
    clampPos = -150; //262
    break;
  case NUM_2:
    clampPos = 0; //277 stuck
    break;
  case NUM_3:
    clampPos = 250; //335
    break;
    
  case LEFT_ARROW:
    blueMotorPosMode = false;
    break;
  case RIGHT_ARROW:
    blueMotorPosMode = true;
    break;
  case NUM_4:
    blueMotorPos = 0;
    break;
  case NUM_7:
    blueMotorPos = targetSonar;
    break;
  case NUM_5:
    blueMotorPos = target45;
    break;
  case NUM_8:
    blueMotorPos = target45_2;
    break;
  case NUM_6:
    blueMotorPos = target25;
    break;
  case NUM_9:
    blueMotorPos = target25_2;
    break;

  default:
    return false;
  }
  return true;
}

/**
 * @brief Determines if the robot's battery level is at a safe level to operate and plays a buzzer tone if they need to be replaced
 * 
 * @return true if the battery voltage is accepetable
 * @return false if the batteries need to be replaced
 */
bool batteryCheck() {
  //Battery check, stops everything and enables buzzer
  if(readBatteryMillivolts() < 6500 && readBatteryMillivolts() > 5000) {
    Serial.println(readBatteryMillivolts());
    blueMotor.setEffort(0);
    clampMotor.reset();
    tone(6, 500);
    return false;
  }
  noTone(6);
  return true;
}

/**
 * @brief Follows a black line on the field with a certain effort
 * 
 * @param effort the effort to line follow with
 */
void followLine(int effort) {
  int left = reflectanceSensor.readLeft();
  int right = reflectanceSensor.readRight();
  int delta = (left - right) / 20;
  chassis.setMotorEfforts(effort+delta, effort-delta);
}
/**
 * @brief Follows a black line on the field until the sonar detects that it is a certain distance away
 * 
 * @param distance the distance to approach to
 * @return true if the robot is the specified distance away from an object
 * @return false if the robot has yet to approach the target distance
 */
bool followLineWithSonar(float distance)
{
  Serial.print("sonar: ");
  Serial.println(sonar.getDistance());
  int effort = -20;
  float delta;
  delta = (sonar.getDistance() - distance);
  if (delta > 0)
  {
    effort = effort * delta;
    if (abs(effort) > 0 && abs(effort) < 70)
    {
      effort = 70 * (effort > 0 ? 1 : -1);
    }
    if (abs(effort) > 100)
    {
      effort = 100 * (effort > 0 ? 1 : -1);
    } // efort hard cap so it doesnt ram
    Serial.print("effort: ");
    Serial.println(effort);
    followLine(effort);
    return false;
  }
  else
  {
    chassis.setMotorEfforts(0, 0);
    return true;
  }
}
/**
 * @brief Method that is continously run when robot is in the "ApproachingRoof" state.
 * This method will bring the robot to the required distance away from the roof to ensure that it can place the collector.
 * 
 * @return true if the robot is at the required distance away from the house
 * @return false if it is still moving towards the house
 */
bool approachRoof()
{
  
  if(roofState == 45)
  {
    return followLineWithSonar(sonar45);
  }
  else if(roofState == 25)
  {
    return followLineWithSonar(sonar25);
  }
  else
  {
    Serial.println("Invalid Roof State Set");
  }
}
/**
 * @brief Method that is ocntinously run when the robot is in the "ApproachingStagingArea" state
 * This method moves the robot using the field lines towards the staging area
 * @return true if the robot is at the required distance away from the staging area
 * @return false if the robot is still moving towards the staging area
 */
bool approachStagingArea()
{
  followLineWithSonar(sonarPickup);
}

void setup() {
  Serial.begin(9600);
  
  chassis.init();

  decoder.init();
  sonar.setup();
  blueMotor.setup();
  clampMotor.setup();

  reflectanceSensor.setup();

  sonar.start();
  robotState = Idle;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool placeRoof()
{

}

void loop() {
  if(!batteryCheck()) return;
  sonar.update(); //Update Sonar
  checkRemote();

  //temp here for manual control of the arm
  // Clamp move to
  clampMotor.moveTo(clampPos);

  //Swaps between move to and direct effort control
  if(blueMotorPosMode) blueMotor.moveTo(blueMotorPos);
  else blueMotor.setEffort(bme);
}