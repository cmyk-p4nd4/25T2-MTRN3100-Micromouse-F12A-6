#pragma once

#include <math.h>
#include "PIDController.hpp"

namespace mtrn3100 {

class offsetController {
public:
    offsetController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), lidController(kp, ki, kd){
        lidController.zeroAndSetTarget(50.0f, 0.0f); // center offset target
    }

    // Computes the output signal required from the current/actual value.
    float compute(float leftRange, float rightRange, float y) {
        if (leftRange < 100 && rightRange < 100) {
            range = 50 + (leftRange - rightRange); // center error: positive if too right
            offset = lidController.compute(range);
        } 
        else if (leftRange < 100) {
            range = leftRange;
            offset = -lidController.compute(range); // invert to push away from left
        } 
        else if (rightRange < 100) {
            range = rightRange;
            offset = lidController.compute(range); // push away from right
        } 
        else {
            range = 250; // no valid wall info
            if (count <= max_counts) {
                count++;
            } else {
                range = 50 + y;
                offset = lidController.compute(range);
            }
        }

        return offset;
    }

    void tune(float p, float i, float d) {
        lidController.tune(p, i, d);
    }

private:
    float kp, ki, kd;
    float offset = 0;
    float enc_offset = 0;
    float range = 250;
    int count = 0;
    int max_counts = 3;
    PIDController lidController;
};

} // namespace mtrn3100
