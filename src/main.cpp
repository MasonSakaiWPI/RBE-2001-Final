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

int pos = ADCMAX / 2;

bool checkRemote() {
  switch (decoder.getKeyCode())
  {
  case NUM_0_10:
    pos = 0;
    break;
  case NUM_1:
    pos = ADCMAX / 9;
    break;
  case NUM_2:
    pos = ADCMAX * 2 / 9;
    break;
  case NUM_3:
    pos = ADCMAX / 3;
    break;
  case NUM_4:
    pos = ADCMAX * 4 / 9;
    break;
  case NUM_5:
    pos = ADCMAX * 5 / 9;
    break;
  case NUM_6:
    pos = ADCMAX * 2 / 3;
    break;
  case NUM_7:
    pos = ADCMAX * 7 / 9;
    break;
  case NUM_8:
    pos = ADCMAX * 8 / 9;
    break;
  case NUM_9:
    pos = ADCMAX;
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
}

void loop() {
  if(readBatteryMillivolts() < 6500 && readBatteryMillivolts() > 100) {
    blueMotor.setEffort(0);
    clampMotor.setEffort(0);
    sonar.stop();
    tone(6, 200);
    return;
  }

  sonar.update();

  checkRemote();
  clampMotor.moveTo(pos);
  
  Serial.print(pos);
  Serial.print("\t");
  Serial.println(analogRead(0));
}