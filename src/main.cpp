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

enum currentRobotStates {Idle, Manual, Initializing, ApproachingRoof, ApproachingStagingArea, PlacingRoof, RemovingRoof, PlacingStagingArea, RemovingStagingArea};
int currentRobotState;
int nextRobotState;
int roofState = 45; //this stores which side of the field the robot is on based on the roof angle

int clampPos = 300, //Clamp target position
    blueMotorPos = 0; //Blue Motor target position
float bme = 0; //Blue motor manual effort

bool blueMotorPosMode = false, //Use BME (false) or blueMotorPos(true)
     clampHolding = false;

const long target45 = -1500,  //45d position
           target45Hover = -1700, // raises plate
           target25 = -3300,  //25d position
           target25Hover = -2500, // raises plate
           targetSonar = -1250, //min for clear of sonar
           targetStagingArea = 0;
const float sonarDropoff = 10.2,
            sonarPickup = 15,
            sonar45 = 13.75,
            sonar25 = 8.3;
const int clampClosed = 80,
          clampOpenSmall = 300,
          clampOpenLarge = 1000;


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
    chassis.setMotorEfforts(0, 0);
    blueMotor.setEffort(0);
    clampMotor.setEffort(0);
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
  int delta = (left - right) / 30;
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
    if (abs(effort) > 0 && abs(effort) < 50)
    {
      effort = 50 * (effort > 0 ? 1 : -1);
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
  if(roofState == 45){return blueMotor.moveTo(clampHolding ? target45Hover : target45) && followLineWithSonar(sonar45);}
  else if(roofState == 25){return blueMotor.moveTo(clampHolding ? target25Hover : target25) && followLineWithSonar(sonar25);}
  else{Serial.println("Invalid Roof State Set");}
  return false;
}
/**
 * @brief Method that is continously run when the robot is in the "ApproachingStagingArea" state
 * This method moves the robot using the field lines towards the staging area
 * @return true if the robot is at the required distance away from the staging area
 * @return false if the robot is still moving towards the staging area
 */
bool approachStagingArea()
{
  return blueMotor.moveTo(targetSonar) && followLineWithSonar(clampHolding ? sonarDropoff : sonarPickup);
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
  currentRobotState = Initializing; //initial state
}
/**
 * @brief Method that is continously run when the robot is in the "PlacingStagingArea" state
 * This method moves the robot using the field lines towards the staging area
 * @return true if the robot has finished placing on the staging area
 * @return false if the robot is still placing on the staging area
 */
bool placeStagingArea()
{
  return blueMotor.moveTo(targetStagingArea) && clampMotor.moveTo(clampOpenSmall);
}
/**
 * @brief Method that is continously run when the robot is in the "RemovingStagingArea" state
 * This method moves the robot using the field lines towards the staging area
 * @return true if the robot has finished removing on the staging area
 * @return false if the robot is still revmoing on the staging area
 */
bool removeStagingArea()
{
  return clampMotor.moveTo(clampClosed) && blueMotor.moveTo(targetSonar); 
}

/**
 * @brief Method that is continously run when the robot is in the "PlacingRoof" state
 * This method places the collector on the roof
 * @return true if the robot has finished placing the panel
 * @return false if the robot is still placing the panel
 */
bool placeRoof()
{
  if(roofState == 45){return blueMotor.moveTo(target45) && clampMotor.moveTo(clampOpenLarge);}
  else if(roofState ==25){return blueMotor.moveTo(target25 && clampMotor.moveTo(clampOpenLarge));}
  else {Serial.println("Invalid Roof State");}
  return false;
}
/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool removeRoof()
{
  if(roofState == 45){return clampMotor.moveTo(clampClosed) && blueMotor.moveTo(target45Hover);}
  else if(roofState == 25){return clampMotor.moveTo(clampClosed) && blueMotor.moveTo(target25Hover);}
  else {Serial.println("Invalid Roof State");}
  return false;
}
void manual()
{
  //Swaps between move to and direct effort control
  if(blueMotorPosMode) blueMotor.moveTo(0);
  else blueMotor.setEffort(bme);
}
void stop()
{
  chassis.setMotorEfforts(0, 0);
  blueMotor.setEffort(0);
  clampMotor.setEffort(0);
}
bool checkRemote() {
  switch (decoder.getKeyCode())
  {
  case PLAY_PAUSE:
    if(currentRobotState == Manual) bme = 0;
    else {
      currentRobotState = nextRobotState;
      nextRobotState = Idle;
    }
    break;
  case VOLminus:
    bme += 50;
    break;
  case VOLplus:
    bme -= 50;
    break;
  case LEFT_ARROW:
    blueMotorPosMode = false;
    break;
  case RIGHT_ARROW:
    blueMotorPosMode = true;
    break;
  case SETUP_BTN:
    blueMotor.reset();
    break;

  case STOP_MODE:
    Serial.print("Robot State: ");
    Serial.println(currentRobotState);
    Serial.print("Clamp State: ");
    Serial.println(clampHolding);
    Serial.print("Blue Motor Encoder: ");
    Serial.println(blueMotor.getPosition());
    Serial.print("Clamp Linear Pot: ");
    Serial.println(clampMotor.getPosition());
    Serial.print("Sonar Dist: ");
    Serial.println(sonar.getDistance());
    Serial.print("Line: ");
    Serial.println(analogRead(3));
    Serial.print("Battery Voltage: ");
    Serial.println(readBatteryMillivolts());
    break;

  case UP_ARROW:
    currentRobotState = Manual;
    blueMotorPosMode = false;
    bme = 0;
    break;
  case ENTER_SAVE:
    currentRobotState = Idle;
    stop();
    break;
    
  case NUM_1:
    currentRobotState = ApproachingRoof;
    break;
  case NUM_4:
    currentRobotState = PlacingRoof;
    break;
  case NUM_7:
    currentRobotState = RemovingRoof;
    break;
  case NUM_3:
    currentRobotState = ApproachingStagingArea;
    break;
  case NUM_6:
    currentRobotState = PlacingStagingArea;
    break;
  case NUM_9:
    currentRobotState = RemovingStagingArea;
    break;
  default:
    return false;
  }
  return true;
}

void loop() {
  if(!batteryCheck()) return;
  sonar.update(); //Update Sonar
  checkRemote();
  switch(currentRobotState)
  {
    case Manual:
      manual();
      break;
    case Initializing:
      if(clampMotor.moveTo(clampOpenLarge)) {
        currentRobotState = Manual;
        nextRobotState = ApproachingRoof;
      }
      break;
    case ApproachingRoof:
      if(approachRoof())
      {
        if(clampHolding){currentRobotState = PlacingRoof;} 
        else {currentRobotState = RemovingRoof;}
      }
      break;
    case PlacingRoof:
      if(placeRoof()) {
        clampHolding = false;
        currentRobotState = Idle;
      }
      break;
    case RemovingRoof:
      if(removeRoof()) {
        clampHolding = true;
        currentRobotState = Idle;
        stop();
      }
      break;
    case ApproachingStagingArea:
      if(approachStagingArea())
      {
        if(clampHolding){currentRobotState = PlacingStagingArea;}
        else currentRobotState = RemovingStagingArea;
      }
      break;
    case PlacingStagingArea:
      if(placeStagingArea()) {
        clampHolding = false;
        nextRobotState = RemovingStagingArea;
        currentRobotState = Idle;
      }
      break;
    case RemovingStagingArea:
      if(removeStagingArea()) {
        clampHolding = true;
        currentRobotState = Idle;
        stop();
      }
      break;
  }
  if(currentRobotState != Manual) blueMotor.safetyCheck();
  clampMotor.safetyCheck();
}