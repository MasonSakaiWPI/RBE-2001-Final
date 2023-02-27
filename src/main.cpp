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

enum robotStates {Idle, ApproachingRoof, ApproachingStagingArea, PlacingRoof, PlacingStagingArea, RemovingRoof};
int robotState;
int roofState = 45; //this stores which side of the field the robot is on based on the roof angle

int clampPos = 300, //Clamp target position
    blueMotorPos = 0; //Blue Motor target position
float bme = 0; //Blue motor manual effort

bool approachRoofTest = false,
     blueMotorPosMode = false; //Use BME (false) or blueMotorPos(true)

const long targetLow = -1500,  //45d position
           targetLow2 = -1700, // raises plate
           targetHigh = -3300,  //25d position
           targetHigh2 = -2500, // raises plate
           targetSonar = -1250; //min for clear of sonar
const float sonarPickup = 10.2,
            sonar45 = 13.3,
            sonar25 = 8.3;



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
    Serial.print("Sonar Dist: ");
    Serial.println(sonar.getDistance());
    Serial.print("Line: ");
    Serial.println(analogRead(3));
    Serial.print("Battery Voltage: ");
    Serial.println(readBatteryMillivolts());
    break;

  case NUM_1:
    clampPos = 0;
    break;
  case NUM_2:
    clampPos = 80;
    break;
  case NUM_3:
    clampPos = 300;
    break;
  case NUM_0_10:
    clampPos = 700;
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
    blueMotorPos = targetLow;
    break;
  case NUM_8:
    blueMotorPos = targetLow2;
    break;
  case NUM_6:
    blueMotorPos = targetHigh;
    break;
  case NUM_9:
    blueMotorPos = targetHigh2;
    break;

  default:
    return false;
  }
  return true;
}

bool batteryCheck() {
  //Battery check, stops everything and enables buzzer
  if(readBatteryMillivolts() < 6500 && readBatteryMillivolts() > 5000) {
    Serial.println(readBatteryMillivolts());
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
bool approachRoof()
{
  
  if(roofState == 45)
  {
    return followLineWithSonar(sonar45);
  }
  else if(roofState = 25)
  {
    return followLineWithSonar(sonar25);
  }
  else
  {
    Serial.println("Invalid Roof State Set");
  }
}
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

void loop() {
  if(!batteryCheck()) return;
  sonar.update(); //Update Sonar
  checkRemote();
  //followLine(100);
  if(approachRoofTest) {approachRoof();}
  else{
    chassis.setMotorEfforts(0,0);
  }
  //followLine(-100);
  switch(robotState)
  {
    case Idle: //waiting for IR remote command
    break;
    case ApproachingRoof:
    break;
  }




  //temp here for manual control of the arm
  // Clamp move to
  if(clampMotor.moveTo(clampPos) == 2) {
    Serial.println(clampMotor.getPosition());
    clampPos = clampMotor.getPosition();
  }

  //Swaps between move to and direct effort control
  if(blueMotorPosMode) blueMotor.moveTo(blueMotorPos);
  else blueMotor.setEffort(bme);
}