#include <Arduino.h>
#include "Ultrasonic.h"
#include <Romi32U4.h>

const byte  pinSonarTrig = 12,
            pinSonarEcho = 17;

const float microsToDistance = 0.01715f;
const unsigned long waitPeriod = 1000,
                    timeoutPeriod = 10000;

unsigned long sonarOut = 0,
            lastSonarTime = 0,
            sonarWaitStart = 0;

bool active = false;
bool pinged = false;

void ping() {
    pinged = true;
    digitalWrite(pinSonarTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinSonarTrig, LOW);
    sonarWaitStart = micros();
}
void echo() {
    if(digitalRead(pinSonarEcho)) {
        sonarOut = micros();
        return;
    }
    lastSonarTime = micros() - sonarOut;
    sonarWaitStart = micros();
    pinged = false;
}

void Setup() {
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  pinMode(17, INPUT);

  attachPCInt(0, echo);
}
void Ultrasonic::setup() { Setup(); }

void Start() {
    active = true;
}
void Ultrasonic::start() { Start(); }
void Stop() {
    active = false;
}
void Ultrasonic::stop() { Stop(); }

void Update() {
    if(active) {
        if(pinged) {
            if(sonarOut + timeoutPeriod <= micros()) ping();
        } else {
            if(sonarWaitStart + waitPeriod <= micros()) ping();
        }
    } else if (pinged) if(sonarOut + timeoutPeriod <= micros()) pinged = false;
}
void Ultrasonic::update() { Update(); }

float GetDistance() {
    if(!active) return -1;
    return lastSonarTime * microsToDistance;
}
float Ultrasonic::getDistance() { return GetDistance(); }