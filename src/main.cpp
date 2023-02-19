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

int pos = 300;
float bme = 0;

bool dbtestactive = false;

const long targetLow = -1000,
           targetHigh = -2729,
           targetGround = 400;


bool checkRemote() {
  switch (decoder.getKeyCode())
  {
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
    Serial.println(blueMotor.getPosition());
    Serial.println(clampMotor.getPosition());
    break;

  case REWIND:
    blueMotor.reset();
    break;

  case NUM_1:
    pos = 0;
    break;
  case NUM_2:
    pos = 80;
    break;
  case NUM_3:
    pos = 300;
    break;
  case NUM_0_10:
    pos = 700;
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

  pinMode(20, INPUT);
  pinMode(21, INPUT);
  pinMode(22, INPUT);

  sonar.start();
}

int lastEffort = 0;

void loop() {
  if(readBatteryMillivolts() < 6500 && readBatteryMillivolts() > 5000) {
    Serial.println(readBatteryMillivolts());
    blueMotor.setEffort(0);
    clampMotor.setEffort(0);
    sonar.stop();
    tone(6, 500);
    return;
  } else noTone(6);

  sonar.update();
  /*Serial.print(sonar.getDistance());
  Serial.print("\t");*/

  checkRemote();

  if(clampMotor.moveTo(pos) == 2) {
    Serial.println(clampMotor.getPosition());
    pos = clampMotor.getPosition();
  }
  
  /*Serial.print(pos);
  Serial.print("\t");
  Serial.println(analogRead(0));*/

  if(dbtestactive && abs(blueMotor.getPosition()) < 100) {
    //blueMotor.setEffort(bme);
    bme -= .1f;
    Serial.print(bme);
    Serial.print("\t");
    Serial.println(blueMotor.getPosition());
  } else if (dbtestactive) {
    dbtestactive = false; bme = 0;
  }
  int dbeffort = 0; blueMotor.setEffort(bme);
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