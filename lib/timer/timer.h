#include <Arduino.h>
class Timer {
    public:
        Timer(int time): time(time), last_time(millis()) {}
        bool isPrimed();
        int time;
    private:
        unsigned long last_time;
};