#include <Arduino.h>
class Timer {
    public:
        Timer(int time): time(time), last_time(micros()) {}
        bool isPrimed();
        int time;
    private:
        unsigned long last_time;
};