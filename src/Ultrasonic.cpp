#include <Arduino.h>
#include "Ultrasonic.h"
#include <Romi32U4.h>

const byte  pinSonarTrig = 12,
            pinSonarEcho = 17;

const float microsToDistance = 0.01715f;
const unsigned long waitPeriod = 1000,
                    timeoutPeriod = 100000;

unsigned long sonarOut = 0,
            lastSonarTime = 0,
            sonarWaitStart = 0;

float distance;

bool active = false;
bool pinged = false;

/**
 * @brief Sends the command to ping to the ultrasonic sensor
*/
void ping() {
    pinged = true;
    digitalWrite(pinSonarTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinSonarTrig, LOW);
    sonarWaitStart = micros();
}
/**
 * @brief Interrupt for sonar ping sent and recived
*/
void echo() {
    if(digitalRead(pinSonarEcho)) {
        sonarOut = micros();
        return;
    }
    lastSonarTime = micros() - sonarOut;
    distance = lastSonarTime * microsToDistance;
    sonarWaitStart = micros();
    pinged = false;
}

/**
 * @brief Sets up the pins and interrupt, and pings the sonar once
*/
void Ultrasonic::setup() {
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  pinMode(17, INPUT);

  attachPCInt(0, echo);
  ping();
}

/**
 * @brief Starts the sonar
*/
void Ultrasonic::start() { active = true; }
/**
 * @brief Stops the sonar, will still update the distance if a ping has already been sent
*/
void Ultrasonic::stop() { active = false; }
/**
 * @brief Gets the active state of the sonar
 * 
 * @return true if the sonar is active, false if it is not
*/
bool Ultrasonic::isActive() { return active; }

void Ultrasonic::update() {
    if(active) {
        if(pinged) {
            if(sonarOut + timeoutPeriod <= micros()) ping();
        } else {
            if(sonarWaitStart + waitPeriod <= micros()) ping();
        }
    } else if (pinged) if(sonarOut + timeoutPeriod <= micros()) pinged = false;
}

/**
 * @brief Gets the last recorded distance of the sonar
 * 
 * @return The distance in unknown units (presumably cm)
*/
float Ultrasonic::getDistance() { return distance; }