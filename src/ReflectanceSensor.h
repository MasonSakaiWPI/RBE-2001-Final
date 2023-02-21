#pragma once
class ReflectanceSensor
{
    public:
        ReflectanceSensor();
        void setup();
        int readLeft();
        int readCenter();
        int readRight();
    private:
};