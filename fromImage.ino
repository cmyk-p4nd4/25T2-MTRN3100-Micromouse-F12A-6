#include "Encoder.hpp"
#include "EncoderOdometry.hpp"
#include "PIDController.hpp"
#include "cubic_spline.h"
#include "Motor.hpp"
#include <Arduino.h>
#include "VL6180X.hpp"
#include <Wire.h>
#include <cmath>
#include <MPU6050_light.h>
#include "move_primitives.h"

typedef struct {
  mtrn3100::VL6180X tof;
  uint32_t last_mea_time;
} Lidar_t;

Lidar_t lidars[3] = {{mtrn3100::VL6180X(), 0},
                     {mtrn3100::VL6180X(), 0},
                     {mtrn3100::VL6180X(), 0}};

PinName pins[3] = {PB_12, PB_13, PB_14};

MPU6050 mpu(Wire);

mtrn3100::Encoder encoder1(TIM1);
mtrn3100::Encoder encoder2(TIM4);
mtrn3100::Motor motor(TIM9, PA_0, PA_1);

enum Direction { NORTH, EAST, SOUTH, WEST };

int start_r = 8;
int start_c = 6; 
Direction start_dir = SOUTH;
int goal_r = 7; 
int goal_c = 5;
std::vector<Cell> movements = {
  {8, 6}, {7, 6}, {7, 7}, {6, 7}, {6, 8}, {5, 8}, {5, 7}, {4, 7}, {4, 8}, {3, 8}, {2, 8}, {2, 7}, {1, 7}, {1, 6}, {0, 6}, {0, 5}, {0, 4}, {0, 3}, {0, 2}, {1, 2}, {2, 2}, {3, 2}, {3, 1}, {2, 1}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {6, 1}, {7, 1}, {7, 2}, {6, 2}, {6, 3}, {7, 3}, {8, 3}, {8, 4}, {8, 5}, {7, 5}
};
bool start_reached = false;


float accu_yaw_angle = 0.0f;
float accu_yaw_time = 0.0f;
unsigned accu_count = 0;
float imu_yaw;


float zero_ref = 0;
float total_yaw = 0;

unsigned long prev_time;

Direction getDirectionFromCells(const Cell& from, const Cell& to) {
    if (to.r == from.r - 1 && to.c == from.c) return NORTH;
    if (to.r == from.r + 1 && to.c == from.c) return SOUTH;
    if (to.c == from.c + 1 && to.r == from.r) return EAST;
    if (to.c == from.c - 1 && to.r == from.r) return WEST;
    return NORTH; // default, no movement
}

int directionDelta(Direction from, Direction to) {
    return (to - from + 4) % 4; // 0=fwd, 1=right, 2=back, 3=left
}

void decideFirstMove(Direction start_dir, const std::vector<Cell>& movements,  float left, float right) {
    if (movements.size() < 2) return; // Need at least two cells

    Direction first_move_dir = getDirectionFromCells(movements[0], movements[1]);
    int delta = directionDelta(start_dir, first_move_dir);

    switch (delta) {
        case 0: // Forward
            start_reached = true;
            break;
        case 1: // Right turn
            move::right_setup(-M_PI_2, right, left);
            break;
        case 3: // Left turn
            move::left_setup(M_PI_2, right, left);
            break;
        case 2: // U-turn
            // Either call special function or use right turn twice
            move::left_setup(M_PI, right, left);
            break;
    }
}


void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.setSCL(PB_8);
  Wire.setSDA(PB_9);
  Wire.begin();

  for (uint8_t i = 0; i < 3; i++) {
    mtrn3100::VL6180X* tof = &lidars[i].tof;
    tof->setBus(&Wire);
    tof->setGPIO0(pins[i]);
    tof->init();
    tof->configureDefault();
    tof->setAddress(0x41 + i);
    tof->startRangeContinuous(50);
    lidars[i].last_mea_time = millis();
  }

  mpu.setFilterGyroCoef(0.98);
  mpu.begin(1, 1);
  mpu.calcOffsets();

  Serial.println("Lets GO!");
  delay(1000);
  mpu.calcGyroOffsets();

  prev_time = millis();

  encoder1.setNegative(false);
  encoder2.setNegative(true);
  delay(1000);
  float left = encoder2.getRotation();
  float right = encoder1.getRotation();
  decideFirstMove(start_dir, movements, left, right);
  delay(1000);
  if (start_reached){
    INIT_SPLINE(left, right, movements, start_r, start_c, start_dir);
  }
}


void loop() {
  // collision avoidance
  volatile float frontRange = 255;
  if (lidars[1].tof.isRangeSampleReady()) {
    frontRange = lidars[1].tof.getRange();
    if (frontRange < 20) {
      motor.setPWM(0,0);
      while (!lidars[1].tof.isRangeSampleReady()) {
        
      } 
      delay(10);
      return;
    }
  }
  volatile float leftRange = 255;
  if (lidars[0].tof.isRangeSampleReady()) {
    leftRange = lidars[0].tof.getRange();
  }
  volatile float rightRange = 255;
  if (lidars[2].tof.isRangeSampleReady()) {
    rightRange = lidars[2].tof.getRange();
  }

  volatile float left = encoder2.getRotation();
  volatile float right = encoder1.getRotation();

  float leftPWM;
  float rightPWM;
  if (!start_reached){
    // update IMU
    mpu.update();
    unsigned long curr_time = micros();
    accu_yaw_angle += mpu.getAngleZ();
    accu_count++;
    if (millis() % 20 == 0) {
      imu_yaw = (accu_yaw_angle / static_cast<float>(accu_count)) * M_PI / 180.f;
      accu_yaw_angle = 0.0f;
      accu_count = 0;
    }
    start_reached = move::compute_move(leftPWM, rightPWM, right, left, frontRange, leftRange, rightRange, imu_yaw);
    if (start_reached){
      INIT_SPLINE(left, right, movements, start_r, start_c, move::g_odom.getH());
    }
  } else {
    compute_pwm(leftPWM, rightPWM, right, left, frontRange, leftRange, rightRange);
  }
  motor.setPWM(rightPWM, leftPWM);
}





  


