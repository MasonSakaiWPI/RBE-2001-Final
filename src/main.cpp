#include <Arduino.h>
#include <Romi32U4.h>
#include <wpi-32u4-lib.h>

#include <Chassis.h>

#include <IRdecoder.h>
#include <ir_codes.h>

#include "Ultrasonic.h"

#include "BlueMotor.h"
#include "ClampMotor.h"

Romi32U4ButtonB buttonB;

IRDecoder decoder(14);

Ultrasonic sonar;

BlueMotor blueMotor;
ClampMotor clampMotor;

int clampPos = 300, //Clamp target position
    blueMotorPos = 0; //Blue Motor target position
float bme = 0; //Blue motor manual effort

bool dbtestactive = false, //Dead band test active
     blueMotorPosMode = false; //Use BME (false) or blueMotorPos(true)

const long targetLow = -1500,  //45d position
           targetLow2 = -1700, // raises plate
           targetHigh = -3250,  //25d position
           targetHigh2 = -1500, // raises plate
           targetHighLower = -500, // past point of drop
           targetSonar = -1250; //min for clear of sonar


bool checkRemote() {
  switch (decoder.getKeyCode())
  { //Everything here is for testing purposes
  case PLAY_PAUSE:
    dbtestactive = !dbtestactive;
    if(dbtestactive) blueMotor.reset();
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
    
  case REWIND:
    blueMotorPos = targetHighLower;
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

void setup() {
  Serial.begin(9600);
  
  decoder.init();
  sonar.setup();
  blueMotor.setup();
  clampMotor.setup();

  //https://www.pololu.com/category/123/pololu-qtr-reflectance-sensors
  //Specific sensor: https://www.pololu.com/product/4246

  pinMode(20, INPUT); //Analog 2, sensor 11
  pinMode(21, INPUT); //Analog 3, sensor 9
  pinMode(22, INPUT); //Analog 4, sensor 7

  sonar.start();
}

int lastEffort = 0;

void loop() {
  //Battery check, stops everything and enables buzzer
  if(readBatteryMillivolts() < 6500 && readBatteryMillivolts() > 5000) {
    Serial.println(readBatteryMillivolts());
    blueMotor.setEffort(0);
    clampMotor.setEffort(0);
    sonar.stop();
    tone(6, 500);
    return;
  } else noTone(6);

  sonar.update(); //Update Sonar
  /*Serial.print(sonar.getDistance());
  Serial.print("\t");*/

  checkRemote();

  // Clamp move to
  if(clampMotor.moveTo(clampPos) == 2) {
    Serial.println(clampMotor.getPosition());
    clampPos = clampMotor.getPosition();
  }
  
  /*Serial.print(pos);
  Serial.print("\t");
  Serial.println(analogRead(0));*/

  // Blue motor test
  if(dbtestactive && abs(blueMotor.getPosition()) < 100) {
    //blueMotor.setEffort(bme);
    bme -= .1f;
    Serial.print(bme);
    Serial.print("\t");
    Serial.println(blueMotor.getPosition());
  } else if (dbtestactive) {
    dbtestactive = false; bme = 0;
  }

  //Swaps between move to and direct effort control
  if(blueMotorPosMode) blueMotor.moveTo(blueMotorPos);
  else blueMotor.setEffort(bme);

  // I have no idea what this is but I'm not removing it yet
  /*if((int)bme != lastEffort) {
        Serial.print(millis());
        Serial.print("\t");
        Serial.print((int)bme);
        Serial.print("\t");
        Serial.print(dbeffort);
        Serial.print("\t");
        Serial.println(blueMotor.getPosition());
  }*/
}