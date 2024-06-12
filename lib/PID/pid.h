#pragma once
#include <Arduino.h>

class PID {
    public:
        PID(float KP, float KI, float KD) : KP(KP), KI(KI), KD(KD), last_update(micros()) {}
        float update(float error);
    private:
        unsigned long last_update;
        float last_value;
        float KP;
        float KI;
        float KD;
        float cumulative_error;
        float last_error;
};
