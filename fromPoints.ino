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

int start_r = 1;
int start_c = 7; 
Direction start_dir = WEST;
int goal_r = 3; 
int goal_c = 1;

static const float IN_X [] = {1383.0, 1340.0, 1165.0, 1120.0, 994.0, 912.0, 748.0, 668.0, 500.0, 333.0, 273.0};

static const float IN_Y [] =  {273.0, 287.0, 309.0, 313.0, 340.0, 374.0, 443.0, 621.0, 709.0, 651.0, 643.0};

static const int N_IN = sizeof(IN_X) / sizeof(IN_X[0]);

bool start_reached = false;


float accu_yaw_angle = 0.0f;
float accu_yaw_time = 0.0f;
unsigned accu_count = 0;
float imu_yaw;


float zero_ref = 0;
float total_yaw = 0;

unsigned long prev_time;

static inline float wrapPi(float a){
  while (a >  M_PI) a -= 2.0f * M_PI;
  while (a < -M_PI) a += 2.0f * M_PI;
  return a;
}

static inline float dirToYaw(Direction d){
  // Coordinate convention: +X = East (right), +Y = South (down)
  // Yaw measured from +X, CCW positive:
  // EAST=0, NORTH=-pi/2, WEST=pi, SOUTH=+pi/2
  switch (d){
    case EAST:  return 0.0f;
    case NORTH: return -M_PI * 0.5f;
    case WEST:  return  M_PI;          // same as -pi; wrapPi() will normalizelater
    case SOUTH: return  M_PI * 0.5f;
  }
  return 0.0f;
}

void decideFirstMove(Direction start_dir) {
  // Need at least two waypoints to define a first segment
  // (You asked to use IN_X/IN_Y[0..1] specifically.)
  const float dx = IN_X[1] - IN_X[0];
  const float dy = IN_Y[1] - IN_Y[0];

  // Desired heading along the first segment
  const float yaw_des = atan2f(-dy, dx);

  // Current heading from the starting grid direction
  const float yaw_now = dirToYaw(start_dir);

  // Shortest signed angle from current to desired
  float dYaw = wrapPi(yaw_des - yaw_now);

  // If already aligned enough, no turn needed
  const float EPS_ALIGN = 2.0f * M_PI / 180.0f; // 2 degrees
  if (fabsf(dYaw) <= EPS_ALIGN) {
    // aligned with first segment: go straight next
    start_reached = true;   // your original flag indicating "just go forward"
    return;
  }

  Serial.println(dYaw);

  // Turn in place by the shortest angle
  if (dYaw > 0.0f) {
    // CCW = left turn by dYaw radians
    move::left_setup(dYaw, /*right_rot*/0.0f, /*left_rot*/0.0f);
  } else {
    // CW = right turn by |dYaw| radians
    move::right_setup(-dYaw, /*right_rot*/0.0f, /*left_rot*/0.0f);
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
  decideFirstMove(start_dir);
  delay(1000);
  if (start_reached){
    INIT_SPLINE(left, right, IN_X, IN_Y, N_IN, start_r, start_c, start_dir);
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
      INIT_SPLINE(left, right, IN_X, IN_Y, N_IN, start_r, start_c, move::g_odom.getH());
    }
  } else {
    compute_pwm(leftPWM, rightPWM, right, left, frontRange, leftRange, rightRange);
  }
  motor.setPWM(rightPWM, leftPWM);
}





  


