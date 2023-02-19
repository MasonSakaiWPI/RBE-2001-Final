#pragma once

const int CountsPerRotation = 540;

class BlueMotor
{
public:
    BlueMotor();
    void setEffort(int effort);
    int setEffortWithDeadband(int effort);
    bool moveTo(long position);
    long getPosition();
    void reset();
    void setup();

private:
    void setEffort(int effort, bool clockwise);
    int setEffortWithDeadband(int effort, bool clockwise);
    static void isrA();
    static void isrB();
    const int tolerance = 10;
    const int PWMOutPin = 11;
    const int AIN2 = 4;
    const int AIN1 = 13;
    const int ENCA = 0;
    const int ENCB = 1;
    const int kp = 16;
    const int Deadband = 0;
};