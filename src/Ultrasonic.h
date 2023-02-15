#pragma once

struct Ultrasonic {
    void setup();

    void start();
    void stop();
    void update();
    
    float getDistance();
};