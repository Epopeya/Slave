#include "pid.h"


float PID::update(float error) {
    unsigned long current_time = micros();
    float dt = (current_time - last_update) * 1e-6f;

    cumulative_error += error * dt;
    float error_rate = (error - last_error) / dt;
    float output = KP * error + KI * cumulative_error + KD * error_rate;

    // Serial.printf("error: %f, XP: %f, XI: %f, XD: %f, out: %f\n", error, error * KP, KI * cumulative_error, KD * error_rate, output);

    last_update = current_time;
    last_error = error;
    
    return output;
}