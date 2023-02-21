#pragma once
class ReflectanceSensor
{
    public:
        ReflectanceSensor();
        void setup();
        int readLeft();
        int readFarLeft();
        int readRight();
        bool farLeftOverLine();
    private:
};