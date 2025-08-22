#pragma once

#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    // Computes the output signal required from the current/actual value.
    float compute(float input) {
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        error = setpoint - (input - zero_ref);

        integral = integral + error * dt;

        float raw_derivative = (error - prev_error) / dt;

        // Low-pass filter coefficient (tune alpha, e.g., 0.1)
        const float alpha = 0.1f;

        // Initialize filtered_derivative on first call if needed
        static bool first_call = true;
        if (first_call) {
            filtered_derivative = raw_derivative;
            first_call = false;
        } else {
            filtered_derivative = alpha * raw_derivative + (1 - alpha) * filtered_derivative;
        }

        output = kp * error + ki * integral + kd * filtered_derivative;

        prev_error = error;

        return output;
    }
    
    void setTarget(float target) {
        setpoint = target;
    }

    // Function used to return the last calculated error. 
    // The error is the difference between the desired position and current position. 
    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    float getError() {
      return error;
    }

    // This must be called before trying to achieve a setpoint.
    // The first argument becomes the new zero reference point.
    // Target is the setpoint value.
    void zeroAndSetTarget(float zero, float target) {
        prev_time = micros();
        zero_ref = zero;
        setpoint = target;
    }

    void updateTarget(float additional) {
        setpoint += additional;
    }

public:
    uint32_t prev_time, curr_time = micros();
    float dt;

private:
    float kp, ki, kd;
    float error, derivative, integral, output;
    float prev_error = 0;
    float setpoint = 0;
    float zero_ref = 0;
    float filtered_derivative = 0.0f;
    
};

}  // namespace mtrn3100
