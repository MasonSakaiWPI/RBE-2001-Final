#pragma once
class ReflectanceSensor
{
    public:
        ReflectanceSensor();
        void setup();
        int* readSensor();
    private:
        static int readings[3];
};