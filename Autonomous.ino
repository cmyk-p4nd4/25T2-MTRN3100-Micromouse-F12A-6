#include "Encoder.hpp"
#include "EncoderOdometry.hpp"
#include "PIDController.hpp"
#include "cubic_spline.h"
#include "Grid.hpp"
#include "Motor.hpp"
#include <Arduino.h>
#include "VL6180X.hpp"
#include <Wire.h>
#include <cmath>
#include <MPU6050_light.h>
#include "move_primitives.h"
#include "oled_maze.hpp"

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

int start_r = 7;
int start_c = 5; 
Direction start_dir = NORTH;
int goal_r = 2; 
int goal_c = 7;
mtrn3100::Grid maze(start_r, start_c, start_dir, goal_r, goal_c);

float accu_yaw_angle = 0.0f;
float accu_yaw_time = 0.0f;
unsigned accu_count = 0;
float imu_yaw;

float zero_ref = 0;
float total_yaw = 0;

unsigned long prev_time;

//global variables for chaining movements
bool exploring = true;
bool racing = false;
char curr_move = 'f';
bool in_move = false;
int curr_r = start_r;
int curr_c = start_c;
Direction curr_dir = start_dir;

OLEDMaze oled(Wire);
uint32_t wall_flags[] = {WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST};


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
  delay(2000);
  float left = encoder2.getRotation();
  float right = encoder1.getRotation();

  oled.init();
  maze.initialiseMaze();

  if (exploring) {
    while (!lidars[1].tof.isRangeSampleReady()) {}
    float frontRange = lidars[1].tof.getRange();
    if (!maze.hasWall(curr_r, curr_c, curr_dir) && frontRange < 90) {
      maze.addWall(curr_r, curr_c, curr_dir);
      oled.cellStatusAt(curr_r, curr_c) |= wall_flags[curr_dir];
      Serial.println("wall added in front");
    }
      
    while (!lidars[0].tof.isRangeSampleReady()) {}
    float leftRange = lidars[0].tof.getRange();
    int dir = static_cast<int>(curr_dir);
    Direction wall_dir = static_cast<Direction>((dir + 3) % 4);
    if (!maze.hasWall(curr_r, curr_c, wall_dir) && leftRange < 90) {
      maze.addWall(curr_r, curr_c, wall_dir);
      oled.cellStatusAt(curr_r, curr_c) |= wall_flags[wall_dir];
      Serial.println("wall added left");
    } 

    while (!lidars[2].tof.isRangeSampleReady()) {}
    float rightRange = lidars[2].tof.getRange();
    dir = static_cast<int>(curr_dir);
    wall_dir = static_cast<Direction>((dir + 1) % 4);
    if (!maze.hasWall(curr_r, curr_c, wall_dir) && rightRange < 90) {
      maze.addWall(curr_r, curr_c, wall_dir);
      oled.cellStatusAt(curr_r, curr_c) |= wall_flags[wall_dir];
      Serial.println("wall added right");
    } 

    curr_move = maze.nextStep(curr_r, curr_c, curr_dir);
    if (curr_move == 'f') {
      move::forward_setup(170, right, left);
    } else if (curr_move == 'r') {
      move::right_setup(-M_PI_2, right, left);
    } else if (curr_move == 'l') {
      move::left_setup(M_PI_2, right, left);
    } else if (curr_move == 'u'){
      move::left_setup(M_PI, right, left);
    } else {
      exploring = false;
    }

      in_move = true;
  }
  oled.draw();
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

  if(millis() % 1000 == 0) {
    float perc = maze.getPercentage();
    char out_str[10];
    snprintf(out_str, sizeof(out_str), "%5.1f%%", perc);
    oled.putText(70, 10, String(out_str));
    oled.draw();
  }

  if (racing) {
    compute_pwm(leftPWM, rightPWM, right, left, frontRange, leftRange, rightRange);
  } else if (exploring) {
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

    if (in_move){
      bool finished = move::compute_move(leftPWM, rightPWM, right, left, frontRange, leftRange, rightRange, imu_yaw);
      if (finished){
        in_move = false;
      }
    } else {
      Serial.println("I am not in move");
      int new_r, new_c;
      Direction new_dir;
      
      maze.applyMove(curr_r, curr_c, curr_dir, curr_move, new_r, new_c, new_dir);
      curr_r = new_r;
      curr_c = new_c;
      curr_dir = new_dir;

      if (curr_move == 'f'){
        while (!lidars[1].tof.isRangeSampleReady()) {}
        frontRange = lidars[1].tof.getRange();
        if (!maze.hasWall(curr_r, curr_c, curr_dir) && frontRange < 90) {
          maze.addWall(curr_r, curr_c, curr_dir);
          oled.cellStatusAt(curr_r, curr_c) |= WALL_EAST | WALL_SOUTH;
          oled.markRobotCell(curr_r, curr_c);
          Serial.println("wall added in front");
        }
          
        while (!lidars[0].tof.isRangeSampleReady()) {}
        float leftRange = lidars[0].tof.getRange();
        int dir = static_cast<int>(curr_dir);
        Direction wall_dir = static_cast<Direction>((dir + 3) % 4);
        if (!maze.hasWall(curr_r, curr_c, wall_dir) && leftRange < 90) {
          maze.addWall(curr_r, curr_c, wall_dir);
          oled.cellStatusAt(curr_r, curr_c) |= wall_flags[wall_dir];
          oled.markRobotCell(curr_r, curr_c);
          Serial.println("wall added left");
        } 

        while (!lidars[2].tof.isRangeSampleReady()) {}
        float rightRange = lidars[2].tof.getRange();
        dir = static_cast<int>(curr_dir);
        wall_dir = static_cast<Direction>((dir + 1) % 4);
        if (!maze.hasWall(curr_r, curr_c, wall_dir) && rightRange < 90) {
          maze.addWall(curr_r, curr_c, wall_dir);
          oled.cellStatusAt(curr_r, curr_c) |= wall_flags[wall_dir];
          oled.markRobotCell(curr_r, curr_c);
          Serial.println("wall added right");
        } 

        curr_move = maze.nextStep(curr_r, curr_c, curr_dir);
        
      } else {
        curr_move = 'f';
        if (maze.returnedHome) {
          exploring = false;
        }
      }
      
      if (curr_move == 'f') {
        move::forward_setup(170, right, left);
      } else if (curr_move == 'r') {
        move::right_setup(M_PI_2, right, left);
      } else if (curr_move == 'l') {
        move::left_setup(M_PI_2, right, left);
      } else if (curr_move == 'u'){
        move::right_setup(M_PI, right, left);
      } else {
        exploring = false;
      }

      in_move = true;
      Serial.print(curr_r); Serial.print(" ,");
      Serial.print(curr_c); Serial.print(" ,");
      Serial.print(curr_dir); Serial.print(" ,");
      Serial.println(curr_move);
      delay(500);
    }

  } else {
    motor.setPWM(0, 0);
    std::vector<Cell> movements = maze.buildCellPath(curr_dir);
    // while (1){
    //   maze.printCosts();
    //   for (int i = 0; i < movements.size(); i++) {
    //     Serial.print("{");
    //     Serial.print(movements[i].r); Serial.print(", ");
    //     Serial.print(movements[i].c); Serial.print("}");
    //   }
    //   Serial.println(".");
    //   delay(1000);
    // }
    INIT_SPLINE(left, right, movements, start_r, start_c, move::g_odom.getH());
    delay(1000);
    racing = true;
  }
  motor.setPWM(rightPWM, leftPWM);
}








  


