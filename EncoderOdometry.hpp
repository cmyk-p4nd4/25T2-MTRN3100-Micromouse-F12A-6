#pragma once

#include <Arduino.h>
#include <math.h>

namespace mtrn3100 {
class EncoderOdometry {
public:
    EncoderOdometry(float radius, float wheelBase) : x(0), y(0), h(0), R(radius), B(wheelBase), lastLPos(0), lastRPos(0) {}

    void update(float leftValue,float rightValue) {

        float delta_left_radians = leftValue - lastLPos; // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEYARE NOT THE WRONG DIRECTION 
        float delta_right_radians = rightValue - lastRPos; // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEY ARE NOT THE WRONG DIRECTION 

        float delta_s = (R * delta_left_radians) / 2 + (R * delta_right_radians) / 2;
        float delta_h = - (R * delta_left_radians) / B + (R * delta_right_radians) / B;

        x += delta_s * cos(h);
        y += delta_s * sin(h);
        h += delta_h;
        
        lastLPos = leftValue;
        lastRPos = rightValue;
    }
    void update2(float leftValue,float rightValue) {

        float delta_left_radians = leftValue - lastLPos; // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEYARE NOT THE WRONG DIRECTION 
        float delta_right_radians = rightValue - lastRPos; // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEY ARE NOT THE WRONG DIRECTION 

        float delta_s = (R * delta_left_radians) / 2 + (R * delta_right_radians) / 2;
        float delta_h = - (R * delta_left_radians) / B + (R * delta_right_radians) / B;

        x += delta_s * cos(h);
        y += delta_s * sin(h);
        h -= delta_h;
        
        lastLPos = leftValue;
        lastRPos = rightValue;
    }
    void reset(float left, float right) {
        lastLPos = left;
        lastRPos = right;
        x = 0;
        y = 0;
        h = 0;
    }


    float getX() const { return x; }
    float getY() const { return y; }
    float getH() const { return h; }

    void setX(float input) { x = input;}
    void setY(float input) { y = input;}
    void setH(float input) { h = input;}

private:
    float x, y, h;
    const float R, B;
    float lastLPos, lastRPos;
};

}
