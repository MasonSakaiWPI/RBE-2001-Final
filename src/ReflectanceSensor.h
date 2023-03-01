#pragma once
class ReflectanceSensor
{
    public:
        ReflectanceSensor();
        void setup();
        int readLeft();
        int readFarRight();
        int readRight();
        bool farRightOverLine();
        bool rightOverLine();
        bool leftOverLine();
        byte updateLeftLineState();
        byte updateRightLineState();
        byte getLeftLineState();
        byte getRightLineState();
    private:
        int LineThreshold = 200;
        byte leftState, rightState;
};