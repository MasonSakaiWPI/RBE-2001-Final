#pragma once

struct Ultrasonic {
    void setup();

    void start();
    void stop();
    void update();

    bool isActive();
    float getDistance();
};