#include "timer.h"

bool Timer::isPrimed() {
    unsigned long current_time = micros();
    if ((current_time - last_time) > time) {
        last_time = current_time;
        return true;
    } else {
        return false;
    }
}