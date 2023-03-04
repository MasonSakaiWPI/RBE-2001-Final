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

enum currentRobotStates
{
  Idle,
  Manual,
  Initializing,
  ApproachingRoof,
  ApproachingStagingArea,
  PlacingRoof,
  RemovingRoof,
  PlacingStagingArea,
  RemovingStagingArea,
  DepartingRoof,
  DepartingStagingArea,
  SwitchSides
};
int currentRobotState = Initializing;
int nextRobotState;
byte departState = 0;
int roofState = 25; // this stores which side of the field the robot is on based on the roof angle
bool resume = false;

float bme = 0; // Blue motor manual effort

bool blueMotorPosMode = false, // Use BME (false) or blueMotorPos(true)
    clampHolding = false;

const long target45 = -1480, // 45d position
    target45Hover = -1700,   // raises plate
    target25 = -3300,        // 25d position
    target25Hover = -2500,   // raises plate
    targetSonar = -1700,     // min for clear of sonar
    targetStagingArea = 0;
const float sonarDropoff = 9.8,
            sonarPickup = 15,
            sonar45 = 13.5,
            sonar45Depart = 17,
            sonar25 = 6.3,
            sonar25Depart = 10,
            sonarSwitch = 5;
const int clampClosed = 70,
          clampOpenSmall = 300,
          clampOpenLarge = 1000;

/**
 * @brief Determines if the robot's battery level is at a safe level to operate and plays a buzzer tone if they need to be replaced
 *
 * @return true if the battery voltage is accepetable
 * @return false if the batteries need to be replaced
 */
bool batteryCheck()
{
  // Battery check, stops everything and enables buzzer
  if (readBatteryMillivolts() < 7000 && readBatteryMillivolts() > 5000)
  {
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
 * 
*/
int followEncoders(int effort, long leftStart, long rightStart) {
  long leftDelta = chassis.getLeftEncoderCount() - leftStart;
  long rightDelta = chassis.getRightEncoderCount() - rightStart;
  long deltaDelta = rightDelta - leftDelta;
  int turnEffort = deltaDelta;
  chassis.setMotorEfforts(effort + turnEffort, effort - turnEffort);
  return deltaDelta;
}
/**
 * @brief Follows a black line on the field with a certain effort
 *
 * @param effort the effort to line follow with
 * @return delta the delta between the two reflectance sensor readings (right-left)
 */
int followLine(int effort)
{
  int left = reflectanceSensor.readLeft();
  int right = reflectanceSensor.readRight();
  int delta = right - left;
  int turnEffort = delta / 30;
  chassis.setMotorEfforts(effort + turnEffort, effort - turnEffort);
  return delta;
}
/**
 * @brief Follows a black line of the field for a certain distance (measured with encoder ticks)
 *
 * @param encoderTicks the amount of encoder ticks to move
 * @return true if the movement has been completed
 * @return false if the movement is still being attempted
 */
bool followLineDistance(long encoderTicks)
{
  long pos = chassis.getLeftEncoderCount();
  long delta = encoderTicks - pos;
  if (abs(delta) < 10)
  {
    chassis.setMotorEfforts(0, 0);
    return true;
  }
  delta *= 10;
  delta = constrain(delta, -70, 70);
  followLine(delta);
  return false;
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
  // Serial.print("sonar: ");
  // Serial.println(sonar.getDistance());
  int effort = -20;
  float delta;
  delta = (sonar.getDistance() - distance);
  if (abs(delta) > .1f)
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
    // Serial.print("effort: ");
    // Serial.println(effort);
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
  if (roofState == 45)
  {
    return blueMotor.moveTo(clampHolding ? target45Hover : target45) && followLineWithSonar(sonar45);
  }
  else if (roofState == 25)
  {
    return blueMotor.moveTo(clampHolding ? target25Hover : target25) && followLineWithSonar(sonar25);
  }
  else
  {
    Serial.println("Invalid Roof State Set");
  }
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

/**
 * @brief Method that is continously run when the robot is in the "PlacingStagingArea" state
 * This method moves the robot using the field lines towards the staging area
 * @return true if the robot has finished placing on the staging area
 * @return false if the robot is still placing on the staging area
 */
bool placeStagingArea()
{
  return blueMotor.moveTo(targetStagingArea) && resume && clampMotor.moveTo(clampOpenSmall);
}
/**
 * @brief Method that is continously run when the robot is in the "RemovingStagingArea" state
 * This method moves the robot using the field lines towards the staging area
 * @return true if the robot has finished removing on the staging area
 * @return false if the robot is still revmoing on the staging area
 */
bool removeStagingArea()
{
  if(clampMotor.moveTo(clampClosed))
    return blueMotor.moveTo(targetSonar);
  blueMotor.setEffort(0);
  return false;
}

/**
 * @brief Method that is continously run when the robot is in the "PlacingRoof" state
 * This method places the collector on the roof
 * @return true if the robot has finished placing the panel
 * @return false if the robot is still placing the panel
 */
bool placeRoof()
{
  if (roofState == 45)
  {
    return blueMotor.moveTo(target45) && clampMotor.moveTo(clampOpenLarge);
  }
  else if (roofState == 25)
  {
    return blueMotor.moveTo(target25) && clampMotor.moveTo(clampOpenLarge);
  }
  else
  {
    Serial.println("Invalid Roof State");
  }
  return false;
}
/**
 * @brief Method that is continiously run when the robot is in the "RemovingRoof" state
 * This method remvoes the collector from the roof
 * @return true
 * @return false
 */
bool removeRoof()
{
  if (roofState == 45)
  {
    if(clampMotor.moveTo(clampClosed)) {
      return resume && blueMotor.moveTo(target45Hover);
    } else {
      blueMotor.setEffort(0);
      return false;
    }
  }
  else if (roofState == 25)
  {
    if(clampMotor.moveTo(clampClosed)) {
      return resume && blueMotor.moveTo(target25Hover);
    } else {
      blueMotor.setEffort(0);
      return false;
    }
  }
  else
  {
    Serial.println("Invalid Roof State");
  }
  return false;
}
void manual()
{
  // Swaps between move to and direct effort control
  if (blueMotorPosMode)
    blueMotor.moveTo(0);
  else
    blueMotor.setEffort(bme);
}
/**
 * @brief Stops all actions being performed on the robot. Halts all motors
 *
 */
void stop()
{
  chassis.setMotorEfforts(0, 0);
  blueMotor.setEffort(0);
  clampMotor.setEffort(0);
}

int linesPassed = -1;
byte turnState = 0;
void resetTurn()
{
  turnState = 0;
  linesPassed = -1;
}
/**
 * @brief
 *
 * @param linesToPass
 * @return true
 * @return false
 */
bool turnLeft(int linesToPass)
{
  switch (turnState)
  {
  case 0: // Wait for Left to rise
    chassis.setMotorEfforts(-70, 70);
    if (reflectanceSensor.getLeftLineState() == RISING)
    {
      linesPassed++;
      turnState = 1;
      if (linesPassed == linesToPass)
        turnState = 3;
    }
    break;
  case 1: // Wait for Right to lower
    if (reflectanceSensor.getRightLineState() == FALLING)
      turnState = 0;
    break;
  case 2:
    if (abs(followLine(0)) > 20)
      return false;
    turnState = 3;
  case 3:
    chassis.setMotorEfforts(0, 0);
    return true;
  }
  return false;
}
/**
 * @brief
 *
 * @param linesToPass
 * @return true
 * @return false
 */
bool turnRight(int linesToPass)
{
  switch (turnState)
  {
  case 0: // Wait for Right to rise
    chassis.setMotorEfforts(70, -70);
    if (reflectanceSensor.getRightLineState() == RISING)
    {
      linesPassed++;
      turnState = 1;
      if (linesPassed == linesToPass)
        turnState = 3;
    }
    break;
  case 1: // Wait for Left to lower
    if (reflectanceSensor.getLeftLineState() == FALLING)
      turnState = 0;
    break;
  case 2:
    if (abs(followLine(0)) > 20)
      return false;
    turnState = 3;
  case 3:
    chassis.setMotorEfforts(0, 0);
    return true;
  }
  return false;
}

/**
 * @brief Method that is run when the robot is in the "DepartingRoof" state
 * Departs the house and orients/prepares the robot to approach the staging area
 * @return true if the action is complete
 * @return false if the action is still being performed
 */
bool departRoof()
{
  static long encoderStart = 0;
  switch (departState)
  {
  case 0: // Back Up
    if (followLineWithSonar((roofState == 45) ? sonar45Depart : sonar25Depart))
    {
      departState = 1;
    }
    break;
  case 1: // turn around
    if (turnLeft(0))
    {
      resetTurn();
      departState = 2;
    }
    break;
  case 2: // Line follow to black line
    followLine(-70);
    if (reflectanceSensor.farRightOverLine())
    {
      departState = 3;
      encoderStart = chassis.getLeftEncoderCount();
    }
    break;
  case 3: // Line follow set distance (encoder)
    if (followLineDistance(encoderStart - 650))
    {
      departState = 4;
    }
    break;
  case 4: // Turn
    if (roofState == 45 ? turnRight(0) : turnLeft(0))
    {
      resetTurn();
      departState = 0;
      return true;
    }
    break;
  }
  return false;
}
/**
 * @brief Method that is run when the robot is in the "DepartingStagingArea" state
 * Departs the house and orients/prepares the robot to approach the house
 * @return true if the action is complete
 * @return false if the action is still being performed
 */
bool departStagingArea()
{
  static long encoderStart = 0;
  switch (departState)
  {
  case 0:
    if(blueMotor.getPosition() > target45 + 200) {
      blueMotor.moveTo(target45);
    } else {
      blueMotor.setEffort(0);
      departState++;
      if(sonar.getDistance() > 50) departState++;
      Serial.println(departState);
    }
    break;
  case 1: // turn around
    if (turnRight(0))
    {
      resetTurn();
      departState++;
    }
    break;
  case 2: // Line follow to black line
    followLine(-70);
    if (reflectanceSensor.farRightOverLine())
    {
      departState++;
      encoderStart = chassis.getLeftEncoderCount();
    }
    break;
  case 3: // Line follow set distance (encoder)
    if (followLineDistance(encoderStart - 650))
    {
      departState++;
    }
    break;
  case 4: // Turn
    if (roofState == 45 ? turnLeft(0) : turnRight(0))
    {
      resetTurn();
      departState++;
      departState = 0;
      return true;
    }
    break;
  }
  return false;
}

/**
 * 
*/
bool switchSides() {
  static long leftEncoderStart = 0;
  static long rightEncoderStart = 0;
  switch (departState)
  {
  case 0: //approach Block
    if(followLineWithSonar(sonarSwitch)) {
      departState++;
      if(roofState == 45) chassis.turnFor(-90, 70, false);
      else chassis.turnFor(90, 70, false);
    }
    break;
  case 1:
    if(chassis.checkMotionComplete()) {
      resetTurn();
      departState++;
      leftEncoderStart = chassis.getLeftEncoderCount();
      rightEncoderStart = chassis.getRightEncoderCount();
    }
    break;
  case 2:
    followEncoders(-70, leftEncoderStart, rightEncoderStart);
    if(reflectanceSensor.farRightOverLine()) {
      departState++;
      leftEncoderStart = chassis.getLeftEncoderCount();
    }
    break;
  case 3:
    if (followLineDistance(leftEncoderStart - 650))
    {
      departState++;
    }
    break;
  case 4:
      if(roofState == 45) chassis.setMotorEfforts(80, -80);
      else chassis.setMotorEfforts(-80, 80);
    if(roofState == 45 ? reflectanceSensor.rightOverLine() : reflectanceSensor.leftOverLine()) {
      chassis.setMotorEfforts(0,0);
      if(roofState == 45) roofState = 25;
      else roofState = 45;
      departState = 0;
      return true;
    }
    break;
  }
  return false;
}

/**
 * @brief Checks the remote for the last button pressed and facilitates the respective action
 *
 * @return true if a successfull button press was recieved
 * @return false otherwise
 */
void checkRemote()
{
  switch (decoder.getKeyCode())
  {
  case PLAY_PAUSE:
    if (currentRobotState == Manual)
      bme = 0;
    else
    {
      resume = true;
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
    Serial.println("blueMotorPosMode is now false");
    break;
  case RIGHT_ARROW:
    blueMotorPosMode = true;
    Serial.println("blueMotorPosMode is now true");
    break;
  case SETUP_BTN:
    blueMotor.reset();
    Serial.println("Re-zeroing blue motor");
    break;

  case ENTER_SAVE:
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
    Serial.print("Left Encoder: ");
    Serial.println(chassis.getLeftEncoderCount());
    Serial.print("Battery Voltage: ");
    Serial.println(readBatteryMillivolts());
    break;

  case UP_ARROW:
    currentRobotState = Manual;
    nextRobotState = currentRobotState;
    resume = false;
    blueMotorPosMode = false;
    bme = 0;
    stop();
    Serial.println("Override to Manual");
    Serial.println("blueMotorPosMode is now false");
    Serial.println();
    break;
  case STOP_MODE:
    if(currentRobotState != Manual)
      nextRobotState = currentRobotState;
    currentRobotState = Idle;
    resume = false;
    stop();
    Serial.println("Override to Idle");
    Serial.println();
    break;

  case NUM_0_10:
    roofState = 25;
    Serial.println("Set Side to 25");
    Serial.println();
    break;
  case REWIND:
    roofState = 45;
    Serial.println("Set Side to 45");
    Serial.println();
    break;
  case DOWN_ARROW:
    currentRobotState = DepartingStagingArea;
    Serial.println("Override to departStagingArea");
    Serial.println();
    break;
  }
}
/**
 * @brief Microcontroller Setup Function. This method is run once on startup.
 *
 */
void setup()
{
  Serial.begin(9600);

  chassis.init();

  decoder.init();
  sonar.setup();
  blueMotor.setup();
  clampMotor.setup();

  reflectanceSensor.setup();

  sonar.start();
  currentRobotState = Initializing; // initial state
}
/**
 * @brief Main robot loop
 *
 */
void loop()
{
  if (!batteryCheck())
    return;
  sonar.update(); // Update Sonar
  checkRemote();
  reflectanceSensor.updateLeftLineState();
  reflectanceSensor.updateRightLineState();
  switch (currentRobotState)
  {
  case Idle:
    if (resume)
    {
      currentRobotState = nextRobotState;
      nextRobotState = Idle;
      resume = false;
      Serial.println("Overriding to nextRobotState");
      Serial.println("Next State is Idle");
      Serial.println();
    }
    break;
  case Manual:
    manual();
    break;
  case Initializing:
    if (clampMotor.moveTo(clampOpenLarge))
    {
      currentRobotState = Manual;
      nextRobotState = ApproachingRoof;
      Serial.println("Moving from Initializing to Manual");
      Serial.println("Next State is ApproachingRoof");
      Serial.println();
    }
    break;
  case ApproachingRoof:
    if (approachRoof())
    {
      if (clampHolding)
      {
        currentRobotState = PlacingRoof;
        Serial.println("Moving from ApproachingRoof to PlacingRoof");
        Serial.println();
      }
      else
      {
        currentRobotState = RemovingRoof;
        Serial.println("Moving from ApproachingRoof to RemovingRoof");
        Serial.println();
      }
      stop();
    }
    break;
  case PlacingRoof:
    if (placeRoof())
    {
      clampHolding = false;
      currentRobotState = DepartingRoof;
      stop();
      Serial.println("Moving from PlacingRoof to DepartingRoof");
      Serial.println("Now Not Holding");
      Serial.println();
    }
    break;
  case RemovingRoof:
    if (removeRoof())
    {
      resume = false;
      clampHolding = true;
      currentRobotState = DepartingRoof;
      stop();
      Serial.println("Moving from RemovingRoof to DepartingRoof");
      Serial.println("Now Holding");
      Serial.println();
    }
    break;
  case DepartingRoof:
    if (departRoof())
    {
      if (clampHolding)
      {
        currentRobotState = ApproachingStagingArea;
        Serial.println("Moving from DepartingRoof to ApproachingStagingArea");
        Serial.println();
      }
      else
      {
        currentRobotState = SwitchSides;
        Serial.println("Moving from DepartingRoof to SwitchSides");
        Serial.println();
      }
    }
    break;
  case ApproachingStagingArea:
    if (approachStagingArea())
    {
      if (clampHolding)
      {
        currentRobotState = PlacingStagingArea;
        Serial.println("Moving from ApproachingStagingArea to PlacingStagingArea");
        Serial.println();
      }
      else
      {
        currentRobotState = RemovingStagingArea;
        Serial.println("Moving from ApproachingStagingArea to RemovingStagingArea");
        Serial.println();
      }
    }
    break;
  case PlacingStagingArea:
    if (placeStagingArea())
    {
      clampHolding = false;
      resume = false;
      nextRobotState = RemovingStagingArea;
      currentRobotState = Idle;
      Serial.println("Moving from PlacingStagingArea to Idle");
      Serial.println("Next State is RemovingStagingArea");
      Serial.println("Now Not Holding");
      Serial.println();
    }
    break;
  case RemovingStagingArea:
    if (removeStagingArea())
    {
      clampHolding = true;
      currentRobotState = DepartingStagingArea;
      stop();
      Serial.println("Moving from RemovingStagingArea to DepartingStagingArea");
      Serial.println("Now Holding");
      Serial.println();
    }
    break;
  case DepartingStagingArea:
    if (departStagingArea())
    {
      currentRobotState = ApproachingRoof;
      Serial.println("Moving from DepartingStagingArea to ApproachingRoof");
      Serial.println();
    }
    break;
    case SwitchSides:
      if(switchSides()) {
        currentRobotState = Idle;
        stop();
        Serial.println("Moving from SwitchSides to Idle");
        Serial.println();
      }
  }
  if (currentRobotState != Manual)
    blueMotor.safetyCheck();
  clampMotor.safetyCheck();
}