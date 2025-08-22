// move_primitives.h
#pragma once
#include <Arduino.h>
#include <cmath>
#include "EncoderOdometry.hpp"
#include "PIDController.hpp"

// ---------------- User-tweakable defaults ------------------------------------
#ifndef MP_WHEEL_R_MM
#define MP_WHEEL_R_MM 22.6f
#endif
#ifndef MP_TRACK_B_MM
#define MP_TRACK_B_MM 89.8f
#endif

#ifndef MP_PWM_MIN
#define MP_PWM_MIN  700.0f
#endif
#ifndef MP_PWM_MAX
#define MP_PWM_MAX  5000
#endif

// Forward (PWM-shaped speed profile)
#ifndef MP_FWD_ACCEL
#define MP_FWD_ACCEL  2500.0f    // PWM/s
#endif
#ifndef MP_FWD_JERK
#define MP_FWD_JERK   5000.0f    // PWM/s^2
#endif
#ifndef MP_FWD_PWM_TARGET
#define MP_FWD_PWM_TARGET 5000.0f
#endif

// Turn controller
#ifndef MP_TURN_KP
#define MP_TURN_KP  500.0f        // PWM per rad of yaw error
#endif
#ifndef MP_TURN_KD
#define MP_TURN_KD  500.0f
#endif
#ifndef MP_TURN_ACCEL
#define MP_TURN_ACCEL  20000.0f    // PWM/s
#endif
#ifndef MP_TURN_EPS
#define MP_TURN_EPS  (0.1f * M_PI/180.0f)
#endif

// Wall straightening (from side LIDARs)
#ifndef MP_WALL_K
#define MP_WALL_K  1000.0f         // PWM per mm of (left-right)
#endif
#ifndef MP_WALL_MAX
#define MP_WALL_MAX 3000.0f
#endif
#ifndef MP_WALL_VALID_MAX
#define MP_WALL_VALID_MAX 70.0f
#endif

// Odometry line-hold (use Y when traveling roughly east/west; X when north/south)
#ifndef MP_ODOM_K
#define MP_ODOM_K  500.0f         // PWM per mm of lateral line error
#endif
#ifndef MP_ODOM_MAX
#define MP_ODOM_MAX 1000.0f
#endif
#ifndef MP_LINE_ALIGN_DEG
#define MP_LINE_ALIGN_DEG  35.0f  // must be within ± this of axis to enable line-hold
#endif

// Blend between LIDAR and ODOM trimming (sum of weights does not need to be 1.0)
#ifndef MP_TRIM_W_LIDAR
#define MP_TRIM_W_LIDAR  2.6f
#endif
#ifndef MP_TRIM_W_ODOM
#define MP_TRIM_W_ODOM   0.9f
#endif

// Early stop when something is too close in front (during forward)
#ifndef MP_FRONT_STOP_MM
#define MP_FRONT_STOP_MM 20.0f
#endif

// --- Forward velocity-sync + straightening tunables ---
#ifndef MP_SYNC_KP
#define MP_SYNC_KP  0.05f        // velocity sync gain (wheel-to-wheel)
#endif
#ifndef MP_SYNC_MIN
#define MP_SYNC_MIN 0.30f        // min scale factor for each side
#endif
#ifndef MP_SYNC_MAX
#define MP_SYNC_MAX 1.00f        // max scale factor for each side
#endif

// Optional: blend LIDAR + ODOM straightening into a single "straighten" term
#ifndef MP_STRAIGHT_K_LIDAR
#define MP_STRAIGHT_K_LIDAR  1.0f   // dimensionless mixer for LIDAR trim into PWM
#endif
#ifndef MP_STRAIGHT_K_ODOM
#define MP_STRAIGHT_K_ODOM   1.0f   // dimensionless mixer for ODOM trim into PWM
#endif

// Internal forward-velocity state
static bool  g_vel_init = false;
static float g_prev_rot_L = 0.0f, g_prev_rot_R = 0.0f;   // previous encoder "rotation" units
static float g_prev_wL    = 0.0f, g_prev_wR    = 0.0f;   // previous wheel velocities (same units/s)
static uint32_t g_last_check_us = 0;                     // 50ms cadence sampler

// Ramp state mirroring your example
static float g_acc_step = 0.0f;      // current accel capability (ramps via jerk)
static float g_last_PWM = 0.0f;      // last commanded base PWM
static bool  no_lidar   = true;      // your flag reused
float g_scaleR = 1.0f;
float g_scaleL = 1.0f;


namespace move {

// ----------------------------------------------------------------------------
// Internal state
// ----------------------------------------------------------------------------
enum class Mode : uint8_t { Idle, Forward, TurnLeft, TurnRight, UTurn, TURN };
enum class Axis : uint8_t { Unknown, X, Y };

static Mode   g_mode        = Mode::Idle;
static bool   g_init_time   = false;
static uint32_t g_prev_us   = 0;

static float g_pwm          = 0.0f;   // current PWM magnitude
static float g_acc          = 0.0f;   // current accel limit (ramping via jerk)

// forward goal
static float g_forward_goal_mm = 0.0f;
static float g_forward_start_x = 0.0f;
static float g_forward_start_y = 0.0f;

// line-hold during forward
static Axis  g_hold_axis   = Axis::Unknown;
static float g_hold_value  = 0.0f;     // Y (if axis X) or X (if axis Y) captured at setup

// turn goal
static float g_turn_goal_rad   = 0.0f;
static float g_yaw_start       = 0.0f;
static float g_last_err        = 0.0f;

// Continuous odometry instance (never reset between moves)
static mtrn3100::EncoderOdometry g_odom(MP_WHEEL_R_MM, MP_TRACK_B_MM);
static bool g_odom_initialized = false; // one-time lazy init against first encoder readings

mtrn3100::PIDController pidController(3000.0f, 5.0f, 50.0f);

// helpers
static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static inline float wrapPi(float a) {
  while (a > M_PI) {
    a -= 2 * M_PI;
  }
  while (a < -M_PI) {
    a += 2 * M_PI;
  }
  return a;                      // result in (−π, π]
}
static inline float deg(float r){ return r * 180.0f / M_PI; }
static inline float rad(float d){ return d * M_PI / 180.0f; }
// decide which axis to hold based on heading at setup
static inline Axis pickHoldAxis(float heading_rad) {
  const float tol = rad(MP_LINE_ALIGN_DEG);
  float dX = fminf(fabsf(wrapPi(heading_rad - 0.0f)), fabsf(wrapPi(heading_rad - (float)M_PI)));
  float dY = fabsf(wrapPi(heading_rad - (float)M_PI_2));
  if (dX <= tol) return Axis::X; // traveling roughly east/west → hold Y
  if (dY <= tol) return Axis::Y; // traveling roughly north/south → hold X
  return Axis::Unknown;
}
// update odometry & dt (lazy one-time baseline)
static inline float updateOdomAndDt(float left_rot, float right_rot) {
  if (!g_odom_initialized) {
    g_odom.reset(left_rot, right_rot);        // one-time align to current encoders
    g_odom_initialized = true;
  } else {
    g_odom.update(left_rot, right_rot);
  }

  uint32_t now = micros();
  if (!g_init_time) { g_prev_us = now; g_init_time = true; }
  float dt = (now - g_prev_us) * 1e-6f;
  if (dt <= 0) dt = 1e-3f;
  g_prev_us = now;
  return dt;
}

// ----------------------------------------------------------------------------
// API: setups (no odometry reset; we snapshot pose/heading instead)
// ----------------------------------------------------------------------------
inline void forward_setup(int dist_mm, float /*right_rot*/, float /*left_rot*/) {
  // Snapshot current pose / axis; keep continuous odometry
  g_forward_goal_mm = (float)dist_mm;
  g_forward_start_x = g_odom.getX();
  g_forward_start_y = g_odom.getY();

  float h = g_odom.getH();
  g_hold_axis = pickHoldAxis(h);
  if (g_hold_axis == Axis::X) {
    g_hold_value = g_odom.getY();   // hold Y while traveling along X
  } else if (g_hold_axis == Axis::Y) {
    g_hold_value = g_odom.getX();   // hold X while traveling along Y
  } else {
    g_hold_value = 0.0f;            // disabled
  }

  // Reset forward control state (but NOT odometry)
  g_mode       = Mode::Forward;
  g_pwm        = 0.0f;
  g_acc        = 0.0f;
  g_acc_step   = 0.0f;
  g_last_PWM   = 0.0f;
  g_init_time  = false;
  no_lidar     = true;

  // Velocity sampler will lazily initialize on first compute_move tick
  g_vel_init = false;
  g_prev_wL = g_prev_wR = 0.0f;
  g_scaleR = 1.0f;
  g_scaleL = 1.0f;
}


inline void right_setup(float radians, float /*right_rot*/, float /*left_rot*/) {
  g_yaw_start     = g_odom.getH();           // snapshot current absolute yaw
  g_turn_goal_rad = -fabsf(radians);         // CW negative
  g_last_err      = 0.0f;
  pidController.zeroAndSetTarget(g_odom.getH(), g_turn_goal_rad);
  Serial.println(g_odom.getH());
  g_mode = Mode::TurnRight;
  g_pwm  = 0.0f;
  g_acc  = 0.0f;
  g_init_time = false;
}

inline void left_setup(float radians, float /*right_rot*/, float /*left_rot*/) {
  g_yaw_start     = g_odom.getH();           // snapshot current absolute yaw
  g_turn_goal_rad =  fabsf(radians);         // CCW positive
  g_last_err      = 0.0f;
  pidController.zeroAndSetTarget(g_odom.getH(), g_turn_goal_rad); // sets left 
  Serial.println(g_odom.getH());
  g_mode = Mode::TurnLeft;
  g_pwm  = 0.0f;
  g_acc  = 0.0f;
  g_init_time = false;
}

// ----------------------------------------------------------------------------
// API: main stepper (returns true when finished)
// ----------------------------------------------------------------------------
inline bool compute_move(
  float& leftPWM, float& rightPWM,
  float right_rot, float left_rot,
  float frontRange, float leftRange, float rightRange,
  float /*imu_yaw*/
){
  leftPWM = rightPWM = 0;

  // NOTE: g_odom expects L,R in this order
  float dt = updateOdomAndDt(left_rot, right_rot);

  switch (g_mode) {
    case Mode::Idle:
      return true;

    case Mode::Forward: {
      // ---- distance since start snapshot (for goal/decel logic) ----
      float dx = g_odom.getX() - g_forward_start_x;
      float dy = g_odom.getY() - g_forward_start_y;
      float s  = g_hold_axis == Axis::X ? fabsf(dx) : fabs(dy);

      // ---- periodic wheel-velocity estimator (~50 ms) ----
      uint32_t now_us = micros();
      if (!g_vel_init) {
        // Align to current encoder readings once
        g_prev_rot_L   = left_rot;
        g_prev_rot_R   = right_rot;
        g_last_check_us = now_us;
        g_vel_init = true;
      }

      float wL = g_prev_wL;   // default to last if 50ms not elapsed
      float wR = g_prev_wR;

      if ((now_us - g_last_check_us) > 50000UL) {
        float dtw = (now_us - g_last_check_us) * 1e-6f;
        if (dtw < 1e-4f) dtw = 1e-4f;

        // "rotation" units are whatever your encoders expose; this preserves that
        float curL = left_rot;
        float curR = right_rot;

        wL = (curL - g_prev_rot_L) / dtw;   // wheel L velocity (units/s)
        wR = (curR - g_prev_rot_R) / dtw;   // wheel R velocity (units/s)

        // wheel angular acceleration (unused but kept for parity with your example)
        float aL = (wL - g_prev_wL) / dtw;
        float aR = (wR - g_prev_wR) / dtw;
        (void)aL; (void)aR;

        // stash
        g_prev_wL = wL; g_prev_wR = wR;
        g_prev_rot_L = curL; g_prev_rot_R = curR;
        g_last_check_us = now_us;
      }

      // ---- base PWM target with jerk/accel ramp (mirrors your example) ----
      float PWM_target = MP_FWD_PWM_TARGET;

      // early stop if obstacle
      if (frontRange > 0 && frontRange < MP_FRONT_STOP_MM) PWM_target = 0.0f;

      // decel near goal (last 60 mm)
      const float slow = 60.0f;
      float remain = clampf(g_forward_goal_mm - s, -1e6f, 1e6f);
      if (remain < slow) PWM_target = 2000;

      // jerk → accel_step (two-stage like your code)
      if (g_acc_step < MP_FWD_ACCEL && g_last_PWM < 500.0f) {
        g_acc_step += MP_FWD_JERK * dt;
      } else if (g_acc_step < MP_FWD_ACCEL) {
        g_acc_step += MP_FWD_JERK * dt;
      } else {
        g_acc_step = MP_FWD_ACCEL;
      }

      // slew-limit base PWM toward target using current accel_step
      float PWM = PWM_target;
      float dPWM = PWM_target - g_last_PWM;
      float maxStep = g_acc_step * dt;
      if      (dPWM >  maxStep) PWM = g_last_PWM + maxStep;
      else if (dPWM < -maxStep) PWM = g_last_PWM - maxStep;
      g_last_PWM = PWM;

      // ---- wheel velocity sync (starts ONLY after first real sample) ----
      if ((now_us - g_last_check_us) > 50000UL) {
        // we already updated wL/wR above; now it’s safe to run sync
        const float MP_SYNC_KP_local = MP_SYNC_KP;
        float vel_err = (wR - wL);     // positive if right is faster

        // Nudge scales toward reducing the error:
        // If right faster -> decrease right scale slightly, increase left slightly.
        g_scaleR -= MP_SYNC_KP_local * vel_err;
        g_scaleL += MP_SYNC_KP_local * vel_err;

        // Keep them bounded and sane
        g_scaleR = clampf(g_scaleR, MP_SYNC_MIN, MP_SYNC_MAX);
        g_scaleL = clampf(g_scaleL, MP_SYNC_MIN, MP_SYNC_MAX);
      }
      // Optional: small grace period at start to avoid initial arc due to startup friction
      const bool startup_grace = (now_us - g_last_check_us) < 120000UL; // ~120 ms after first stamp
      float apply_scaleR = startup_grace ? 1.0f : g_scaleR;
      float apply_scaleL = startup_grace ? 1.0f : g_scaleL;

      
      // ---- straightening term (LIDAR + line-hold ODOM), like your 'straighten' ----
      // Reuse your earlier trim computations but compress to a single "straighten"
      float trim_lidar;
      bool Lok = (leftRange  > 0 && leftRange  < MP_WALL_VALID_MAX);
      bool Rok = (rightRange > 0 && rightRange < MP_WALL_VALID_MAX);
      if (Lok && Rok) {
        trim_lidar = clampf(MP_WALL_K * (leftRange - rightRange), -MP_WALL_MAX, MP_WALL_MAX);
      } else if (Lok) {
        trim_lidar = clampf(MP_WALL_K * (leftRange - 50), -MP_WALL_MAX, MP_WALL_MAX);
      } else if (Rok) {
        trim_lidar = clampf(MP_WALL_K * (50 - rightRange), -MP_WALL_MAX, MP_WALL_MAX);
      }

      float trim_odom;
      if (g_hold_axis == Axis::X) {
        float errY = g_odom.getY() - g_hold_value; // +ve drift up
        trim_odom = clampf(MP_ODOM_K * errY, -MP_ODOM_MAX, MP_ODOM_MAX);
        // flip sign when heading roughly east (−90..+90 deg) so "positive means push right wheel"
        float h = wrapPi(g_odom.getH());
        if (h <= M_PI_2 && h >= -M_PI_2) trim_odom = -trim_odom;
      } else if (g_hold_axis == Axis::Y) {
        float errX = g_odom.getX() - g_hold_value; // +ve drift right
        trim_odom = clampf(MP_ODOM_K * errX, -MP_ODOM_MAX, MP_ODOM_MAX);
        // flip sign when heading in negative X half-plane (π..0)
        float h = wrapPi(g_odom.getH());
        //if (h >= 0) trim_odom = -trim_odom;
      }

      // Single straighten term, like your 'straightController' output (units: PWM)
      float straighten = MP_STRAIGHT_K_LIDAR * trim_lidar + MP_STRAIGHT_K_ODOM * trim_odom;

      // ---- compose per-side PWMs (very close to your example) ----
      float rightOut = PWM * g_scaleR;
      float leftOut  = PWM * g_scaleL;

      if (straighten > 0) {
        rightOut += straighten;
      } else if (straighten < 0) {
        leftOut  -= straighten; // minus because straighten is negative
      }

      // deadband lift / clamp
      auto lift = [](float p){
        if (p > 0) return clampf(std::max(p, (float)MP_PWM_MIN), -((float)MP_PWM_MAX), (float)MP_PWM_MAX);
        if (p < 0) return clampf(std::min(p, -((float)MP_PWM_MIN)), -((float)MP_PWM_MAX), (float)MP_PWM_MAX);
        return 0.0f;
      };
      rightOut = lift(rightOut);
      leftOut  = lift(leftOut);

      rightPWM = (int)rightOut;
      leftPWM  = (int)leftOut;

      // LIDAR seen?
      if (frontRange > 0 && frontRange < 90.0f) {
        no_lidar = false;
      }
      if (!no_lidar && PWM < 700) {
        rightPWM = 700;
        leftPWM = 700;
      }

      // ---- finish conditions (keep your original semantics) ----
      if ((remain <= 2.0f && no_lidar) || (frontRange > 0 && frontRange <= 60.0f) || !no_lidar && remain <= -3.0f) { //&& fabsf(PWM) <= MP_PWM_MIN 
        rightPWM = 0;
        leftPWM = 0;
        g_mode = Mode::Idle;
        return true;
      }
      return false;
    }


    case Mode::TurnLeft: {
      float error = g_turn_goal_rad - (g_odom.getH() - g_yaw_start);
      float PWM;
      if (error > (M_PI / 6.0f)) {
        PWM = -2000.f;
      } else if (error < -(M_PI / 6.0f)) {
        PWM = 2000.f;
      } else {
        PWM = -pidController.compute(g_odom.getH());
      }
      leftPWM = PWM;
      rightPWM = -PWM;

      return fabsf(error) < 0.04 ? true : false;
    }

    case Mode::TurnRight: {
      float error = g_turn_goal_rad - (g_odom.getH() - g_yaw_start);
      float PWM;
      if (error > (M_PI / 6.0f)) {
        PWM = 2000.f;
      } else if (error < -(M_PI / 6.0f)) {
        PWM = -2000.f;
      } else {
        PWM = pidController.compute(g_odom.getH());
      }
      leftPWM = -PWM;
      rightPWM = PWM;

      return fabsf(error) < 0.04 ? true : false;
    }

  }

  g_mode = Mode::Idle;
  return true;
}

} // namespace move
