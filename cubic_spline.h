#include "EncoderOdometry.hpp"
#include "PIDController.hpp"
#include "waypoints.h"
#include <Arduino.h>
#include <Wire.h>
#include <cmath>
#include <vector>

// ========================= USER CONFIG =========================
// Geometry
static constexpr float WHEEL_R = 22.6f;   // wheel radius [mm]
static constexpr float TRACK_B = 89.8f;  // axle length [mm]

// Motion limits (tune)
static constexpr float V_MAX      = 250.0f;   // max linear speed [mm/s] (cap)
static constexpr float A_LONG_MAX = 5000.0f;  // max longitudinal accel
static constexpr float J_LONG_MAX = 10000.0f; // max longitudinal jerk [mm/s^3]
static constexpr float A_LAT_MAX  = 10000.0f;  // max lateral accel [mm/s^2] → limits speed by curvature
static constexpr float W_MAX      = 12.5f;

// Pure pursuit
static constexpr float LOOKAHEAD  = 100.0f;   // [mm] arc-distance lookahead along spline LEGACY
static constexpr float END_STOP_L  = 10.0f;   // [mm] stop when remaining path < this

// PWM feedforward (tune quickly)
static constexpr float KF_PWM_PER_MMS = 35; // slope: mm/s → PWM LEGACY
static constexpr int   PWM_MIN        = 700;   // overcome stiction
static constexpr int   PWM_MAX        = 10000; // motor limit

static constexpr float MIN_TURN_RADIUS = 0.01f;     // start conservative
static constexpr float MAX_KAPPA = 1.0f / MIN_TURN_RADIUS;

float cell_mm    = 177.0f;
float clearance  = 60.0f;     // keep center at least 50 mm from outer walls
float radius_mm  = 85.0f;     // your exact requested corner radius
float step_mm    = 50.0f;

std::vector<Cell> path = {
  {0,0}, {0,1}, {0,2}, {1,2}, {1,1}, {1,0}
};

// 3) Copy into your WP_X / WP_Y and N_WP before INIT_SPLINE()
int N_WP;
const int MAX_WP = 1000;
float WP_X[MAX_WP];
float WP_Y[MAX_WP];

inline void loadWaypointsFrom(const std::vector<Pt>& wps) {
  N_WP = (int)std::min((size_t)MAX_WP, wps.size());
  for (int i = 0; i < N_WP; ++i) {
    WP_X[i] = wps[i].x;
    WP_Y[i] = -1.0f * wps[i].y;
    Serial.print(WP_X[i]);
    Serial.print(", ");
    Serial.println(WP_Y[i]);
  }
}

// ---- Catmull–Rom (centripetal) with tension ----
static constexpr float ALPHA = 0.5f;   // 0.5 = centripetal (best for avoiding loops/overshoot)
static float tK[MAX_WP];               // knot parameters (not your "t_hat")
static float Tx[MAX_WP], Ty[MAX_WP];   // tangents at each waypoint
static float Tension = 0.75f;           // 0..1 ; 0 = classic CR, 0.5..0.8 = tighten

static inline float dist(const float x0, const float y0, const float x1, const float y1){
  return sqrtf((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));
}

// ================ DEVICES (yours) ==================

// Velocity PIDs (per wheel). Start small; we’ll tune.
mtrn3100::PIDController pidL(0.5f, 0.005f, 0.01f);
mtrn3100::PIDController pidR(0.5f, 0.005f, 0.01f);

mtrn3100::EncoderOdometry odom(WHEEL_R, TRACK_B); 

// Optional: tiny helper to add a setTarget() into your PID class
// --- Add this method to your PIDController class header ---
//   void setTarget(float target) { setpoint = target; }
// And (recommended) low-pass derivative inside PID (as we discussed)

// ============ GLOBALS / STATE ===========
uint32_t t_prev_us;
float last_rot_L = 0.0f, last_rot_R = 0.0f;
float vL_meas = 0.0f, vR_meas = 0.0f; // [mm/s]
float v_cmd = 0.0f;      // commanded linear speed [mm/s]
float v_cmd_prev = 0.0f;
float a_cmd = 0.0f;      // commanded longitudinal accel [mm/s^2]

// ================== SPLINE INTERNALS ==================
// Natural cubic spline over chord-length parameter t[]
// We build two 1D splines: X(t), Y(t)
static constexpr int MAX_N = MAX_WP;
float T_param[MAX_N]; // chord-length parameter
float X2[MAX_N], Y2[MAX_N]; // second derivatives at knots (natural BC)
float total_chord = 0.0f;

// Forward decls
void buildNaturalCubic();
void buildCatmullRom();
void splineEval(float tq, float& x, float& y, float& dx, float& dy, float& ddx, float& ddy);
static void evalCRSeg(int i, float t, float& x, float& y, float& dx, float& dy, float& ddx, float& ddy);

// Track along parameter t (monotonic 0..T_param[N_WP-1])
float t_hat = 0.0f; // our current estimate of where we are on the spline

// ================== UTILS ==================
static inline float normAngle(float a) {
  while (a >  M_PI) a -= 2.0f * M_PI;
  while (a < -M_PI) a += 2.0f * M_PI;
  return a;
}

static inline int pwmFromVelR(float v_mm_s) {
  // simple linear FF + deadband lift; sign-preserving
  if (fabsf(v_mm_s) < 1e-3f) return 0;
  float pwm = (v_mm_s + 10.0f) / 0.0537f;
  if (pwm > 0) pwm = (fabsf(pwm) < PWM_MIN) ? PWM_MIN : pwm;
  else         pwm = (fabsf(pwm) < PWM_MIN) ? -PWM_MIN : pwm;
  pwm = clampf(pwm, -PWM_MAX, PWM_MAX);
  return static_cast<int>(pwm);
}
static inline int pwmFromVelL(float v_mm_s) {
  // simple linear FF + deadband lift; sign-preserving
  if (fabsf(v_mm_s) < 1e-3f) return 0;
  float pwm = (v_mm_s + 10.0f) / 0.0541f;
  if (pwm > 0) pwm = (fabsf(pwm) < PWM_MIN) ? PWM_MIN : pwm;
  else         pwm = (fabsf(pwm) < PWM_MIN) ? -PWM_MIN : pwm;
  pwm = clampf(pwm, -PWM_MAX, PWM_MAX);
  return static_cast<int>(pwm);
}

// Advance parameter by target arc-length (mm) using adaptive steps
float advanceByArc(float t0, float ds_target) {
  float t = clampf(t0, 0.0f, T_param[N_WP-1]);
  float remaining = fmaxf(ds_target, 0.0f);

  // Take up to ~10 mm arc-length per substep (fast + stable)
  const float MAX_DS = 10.0f;   // mm
  const int   MAX_IT = 200;     // safety cap

  for (int i = 0; i < MAX_IT && remaining > 0.0f && t < T_param[N_WP-1]; ++i) {
    float x, y, dx, dy, ddx, ddy;
    splineEval(t, x, y, dx, dy, ddx, ddy);
    float vt = hypotf(dx, dy);            // ds/dt (mm per unit t)
    if (vt < 1e-6f) break;                // avoid div-by-zero / degenerate

    float ds = fminf(MAX_DS, remaining);  // arc to advance this substep
    float dt = ds / vt;                   // convert arc step to param step
    t += dt;
    remaining -= ds;
  }

  return clampf(t, 0.0f, T_param[N_WP-1]);
}

// Find a nearby t minimizing distance to (xr,yr). Search around current t_hat.
float nearestT(float xr, float yr) {
  float best_t = t_hat, best_d2 = 1e30f;

  // Keep the window small and FORWARD from current t_hat
  const float WIN  = 80.0f;  // mm of chord-length
  const float STEP = 2.0f;

  float t0 = clampf(t_hat,               0.0f, T_param[N_WP-1]);
  float t1 = clampf(t_hat + WIN,         0.0f, T_param[N_WP-1]);
  if (t1 - t0 < STEP) t1 = clampf(t0 + STEP, 0.0f, T_param[N_WP-1]);

  for (float t = t0; t <= t1; t += STEP) {
    float x,y,dx,dy,ddx,ddy;
    splineEval(t,x,y,dx,dy,ddx,ddy);
    float d2 = (x-xr)*(x-xr) + (y-yr)*(y-yr);
    if (d2 < best_d2) { best_d2 = d2; best_t = t; }
  }
  return best_t;
}

float speedLimitFrom(float kappa) {
  float k = fabsf(kappa) + 1e-6f;
  float v_yaw = W_MAX / k;                 // honor yaw-rate limit
  float v_lat = sqrtf(A_LAT_MAX / k);      // honor lateral accel
  return fminf(V_MAX, fminf(v_yaw, v_lat));
}

// --- Wall/Front lidar nudger -----------------------------------------------
// Assumptions:
// - Valid lidar readings are < 200 mm (as per your system).
// - When centered in a cell, each side lidar reads ~50 mm to the wall.
// - Positive kappa_cmd turns left (CCW); negative = right (CW).
//   (Matches kappa_cmd = 2*sin(alpha)/Ld sign convention.)
static inline void wallNudge(float leftRange, float rightRange, float frontRange,
                             float &v_cmd, float &kappa_cmd)
{
  // Tunables (start here)
  const float DESIRED_SIDE = 50.0f;     // mm, your “centered” side distance
  const float VALID_MAX    = 180.0f;    // mm, ignore above this (invalid/too far)
  const float DEAD_BAND    = 5.0f;      // mm, ignore tiny noise
  const float K_WALL       = 0.03f;   // -> curvature per mm of side error (tweak)
  const float K_FRONT_TURN = 0.0050f;   // extra curvature bias as front gets close
  const float K_FRONT_SLOW = 0.05f;    // speed scale factor per mm closeness
  const float FRONT_SLOW   = 100.0f;    // start slowing here (mm)
  const float FRONT_STOP   =  30.0f;    // near-stop here (mm)
  const float DELTA_KAPPA_MAX = 1.0f/40.0f; // cap curvature nudge (≈ R=90 mm)

  // 1) Side wall “push” (steer away from nearer wall)
  float delta_kappa = 0.0f;

  bool L_ok = (leftRange  > 0.0f && leftRange  < VALID_MAX);
  bool R_ok = (rightRange > 0.0f && rightRange < VALID_MAX);

  if (L_ok || R_ok) {
    // Lateral bias: positive when right > left -> we’re closer to left wall
    // We want negative curvature (steer right) in that case.
    float e_lat = 0.0f;

    if (L_ok && R_ok) {
      // Use relative difference (strongest & absolute-scale free)
      e_lat = (rightRange - leftRange);
    } else if (L_ok) {
      // Only left valid: compare to desired
      e_lat = (DESIRED_SIDE - leftRange);    // >0 means too close left -> steer right
    } else { // R_ok only
      e_lat = (DESIRED_SIDE - rightRange);  // >0 means too far from right -> steer left
    }

    // Deadband + scale to curvature
    if (fabsf(e_lat) > DEAD_BAND) {
      delta_kappa -= K_WALL * e_lat;  // NOTE the minus: closer-left => steer right (neg)
    }
  }

  // 2) Front slowdown + extra turn bias
  bool F_ok = (frontRange > 0.0f && frontRange < VALID_MAX);
  if (F_ok) {
    // (a) Speed scale
    if (frontRange < FRONT_SLOW) {
      // Scale down as it gets closer; clamp >= 0.1 to avoid total stall unless very close
      float closeness = (FRONT_SLOW - frontRange) / fmaxf(FRONT_SLOW - FRONT_STOP, 1.0f);
      closeness = fminf(fmaxf(closeness, 0.0f), 1.0f);
      float scale = 1.0f - K_FRONT_SLOW * (FRONT_SLOW - frontRange);
      scale = fminf(fmaxf(scale, 0.10f), 1.0f);
      v_cmd *= scale;
    }

    // (b) Extra curvature bias: turn away from nearer side when something is ahead
    if (L_ok || R_ok) {
      float bias = 0.0f;
      if (L_ok && R_ok) {
        // Turn toward the side with more space
        bias = (rightRange - leftRange);   // positive if more room on right -> left turn?
        // We want positive kappa for left turn, so bias directly:
        delta_kappa += K_FRONT_TURN * (bias / (DESIRED_SIDE + 1e-3f));
      } else if (L_ok) {
        // Only left valid: obstacle ahead → prefer turning right (away from left wall)
        delta_kappa += -K_FRONT_TURN;
      } else {
        // Only right valid: obstacle ahead → prefer turning left
        delta_kappa += +K_FRONT_TURN;
      }
    }
  }

  // 3) Smooth & clamp the curvature nudge to avoid twitch
  static float delta_kappa_f = 0.0f;
  const float LPF = 0.25f; // 0..1 (higher = faster)
  delta_kappa_f += LPF * (delta_kappa - delta_kappa_f);

  if (delta_kappa_f >  DELTA_KAPPA_MAX) delta_kappa_f =  DELTA_KAPPA_MAX;
  if (delta_kappa_f < -DELTA_KAPPA_MAX) delta_kappa_f = -DELTA_KAPPA_MAX;

  // 4) Apply
  kappa_cmd += delta_kappa_f;
}




// ================== SETUP ==================
void INIT_SPLINE(float left, float right, std::vector<Cell> movements, int start_r, int start_c, float start_dir) {
  pidL.zeroAndSetTarget(0.0f, 0.0f);
  pidR.zeroAndSetTarget(0.0f, 0.0f);
  odom.reset(left, right);


  // Build chord-length parameter and natural cubic spline
  std::vector<Pt> waypoints = generateWaypointsCenters(
  movements, /*maze_cols=*/9, /*maze_rows=*/9,
  cell_mm, clearance);
  // std::vector<Pt> waypoints = generateWaypointsFixedRadius(
  // movements, /*maze_cols=*/3, /*maze_rows=*/2,
  // cell_mm, clearance, radius_mm, step_mm
  // );

  N_WP = (int)waypoints.size();
  loadWaypointsFrom(waypoints);
  buildCatmullRom();
  // buildNaturalCubic();

  float init_heading = atan2f(WP_Y[1]-WP_Y[0], WP_X[1]-WP_X[0]);
  Serial.print(init_heading); Serial.print(", "); Serial.println(start_dir - M_PI_2);
  odom.setH(start_dir - M_PI_2);
  int start_x = (start_c + 0.5f) * cell_mm;
  odom.setX(start_x);
  int start_y = -1.0f * (start_r + 0.5f) * cell_mm;
  odom.setY(start_y);

  t_hat = 0.0f;
  // Start timing
  t_prev_us = micros();
  last_rot_L = left;
  last_rot_R = right;
  
  // CSV header
  Serial.println(F("t, x, y, h, t_hat, x_ref, y_ref, x_look, y_look, v_cmd, omega, kappa, vL_des, vR_des, vL_meas, vR_meas, pwmL, pwmR, front"));
}

// ================== MAIN LOOP ==================
void compute_pwm(float &left_pwm, float &right_pwm, float rotR, float rotL, volatile float frontRange, volatile float leftRange, volatile float rightRange) {
  // --- dt ---
  uint32_t now = micros();
  float dt = (now - t_prev_us) * 1e-6f;
  if (dt <= 0) dt = 1e-3f;
  t_prev_us = now;

  odom.update(rotL, rotR); // (L, R) order per your class

  // wheel speeds (mm/s): v = R * dθ/dt
  float dthetaL = rotL - last_rot_L;
  float dthetaR = rotR - last_rot_R;
  last_rot_L = rotL;
  last_rot_R = rotR;
  vL_meas = WHEEL_R * (dthetaL / dt);
  vR_meas = WHEEL_R * (dthetaR / dt);

  float xr = odom.getX();
  float yr = odom.getY();
  float hr = odom.getH();


  // --- Where are we on the spline? ---
  t_hat = nearestT(xr, yr);

  // --- Reference point (closest) and tangent/curvature there ---
  float x_ref,y_ref,dx,dy,ddx,ddy;
  splineEval(t_hat, x_ref, y_ref, dx, dy, ddx, ddy);

  // curvature kappa = (x'y'' - y'x'') / |r'|^3 (parameterization invariant)
  float v_t = hypotf(dx, dy); // |r'(t)|
  float kappa = 0.0f;
  if (v_t > 1e-6f) {
    kappa = (dx*ddy - dy*ddx) / (v_t*v_t*v_t);
  }

  float remains = T_param[N_WP-1] - t_hat;
  float Ld_mm = clampf(20.0f + 0.5f*v_cmd, 20.0f, 50.0f);
  float Ld_curve = clampf(0.5f / (fabsf(kappa) + 1e-4f), 20.0f, 50.0f);
  Ld_mm = fminf(Ld_mm, Ld_curve);
  // shrink near the end
  if (remains < 180.0f) Ld_mm = fmaxf(60.0f, 0.5f*remains);

  float t_look = advanceByArc(t_hat, Ld_mm);
    
  float x_look,y_look,dxL,dyL,ddxL,ddyL;
  splineEval(t_look, x_look, y_look, dxL, dyL, ddxL, ddyL);

  // --- Pure pursuit curvature command from geometry ---
  float dxg = x_look - xr;
  float dyg = y_look - yr;
  float Ld = hypotf(dxg, dyg);
  float alpha = normAngle(atan2f(dyg, dxg) - hr);
  float kappa_cmd = 0.0f;
  static constexpr float Ld_MAX = 220.0f;   // ~1–1.5 cells
  float Ld_eff = fmaxf(1e-3f, fminf(Ld, Ld_MAX));
  kappa_cmd = 2.0f * sinf(alpha) / Ld_eff;
  if (!isfinite(kappa_cmd) || (fabsf(alpha) < 0.02f)) {
    kappa_cmd = 0.0f;
  }
  kappa_cmd = clampf(kappa_cmd, -MAX_KAPPA, MAX_KAPPA);

  // --- Speed selection: curvature & lateral accel cap ---
  float v_cap_curve = (fabsf(kappa_cmd) < 1e-5f) ? V_MAX
                     : sqrtf(A_LAT_MAX / fabsf(kappa_cmd)); // v ≤ sqrt(a_lat / |k|)
  float v_target = clampf(v_cap_curve, 0.0f, V_MAX);

  // --- Longitudinal jerk/accel limiting (S-curve-ish) ---
  // Move a_cmd toward 0 with jerk limit; then move v_cmd toward v_target with accel limit.
  float a_des = clampf((v_target - v_cmd) / dt, -A_LONG_MAX, A_LONG_MAX);
  float da = clampf(a_des - a_cmd, -J_LONG_MAX * dt, J_LONG_MAX * dt);
  a_cmd += da;
  a_cmd = clampf(a_cmd, -A_LONG_MAX, A_LONG_MAX);

  v_cmd += a_cmd * dt;
  // Don’t exceed target due to jerk lag
  if ((v_cmd > v_target && a_cmd > 0) || (v_cmd < v_target && a_cmd < 0)) {
    a_cmd = 0.0f;
    v_cmd = v_target;
  }

  if (leftRange < 100 || rightRange < 100 || frontRange < 100) {
    wallNudge(leftRange, rightRange, frontRange, v_cmd, kappa_cmd);
  }

  float v_ref = fminf(v_cmd, speedLimitFrom(kappa_cmd));
  // v_cmd = slewLimit(v_ref, v_cmd_prev, A_LONG_MAX * dt);  // accel/jerk limit

  // after computing v_cmd and dt
  t_hat += clampf(0.1f * v_cmd * dt, 0.0f, 60.0f);  // predict forward in t by ~arc length
  t_hat = clampf(t_hat, 0.0f, T_param[N_WP-1]);
  t_hat = nearestT(xr, yr); // then re-snap to nearest within a large window

  // --- Convert (v, kappa_cmd) → wheel targets ---
  float omega_cmd = v_cmd * kappa_cmd; // [rad/s]
  float vL_des = v_cmd - 0.5f * omega_cmd * TRACK_B;
  float vR_des = v_cmd + 0.5f * omega_cmd * TRACK_B;

  float s = fmaxf(fabsf(vR_des), fabsf(vL_des)) / V_MAX;
  if (s > 1.0f) {
    v_cmd /= s;                  // reduce forward speed
    vR_des = v_cmd + 0.5f*omega_cmd*TRACK_B;   // keep same omega
    vL_des = v_cmd - 0.5f*omega_cmd*TRACK_B;
  }

  // --- Per-wheel control: Feedforward + PID trim ---
  // (1) Base FF
  int pwmL_ff = pwmFromVelL(vL_des);
  int pwmR_ff = pwmFromVelR(vR_des);

  // (2) PID trims (velocity): we update setpoints and compute error = setpoint - input
  // Add this method in PIDController: void setTarget(float target) { setpoint = target; }
  pidL.setTarget(vL_des);
  pidR.setTarget(vR_des);

  static float vL_f=0, vR_f=0; 
  const float BETA=0.25f; // 0..1; higher = more smoothing
  vL_f += BETA*(vL_meas - vL_f);
  vR_f += BETA*(vR_meas - vR_f);
  float trimL = pidL.compute(vL_f);
  float trimR = pidR.compute(vR_f);

  // Optional: scale trims so they’re in PWM units if your PID gains are "velocity → PWM"
  int pwmL = clampf(pwmL_ff + trimL, -PWM_MAX, PWM_MAX);
  int pwmR = clampf(pwmR_ff + trimR, -PWM_MAX, PWM_MAX);

  // --- End-of-path stop ---
  float remaining = T_param[N_WP-1] - t_hat;
  // Roughly convert remaining param to mm: use local |r'| to scale
  float mm_per_t = v_t; // ds/dt ≈ |r'(t)|
  float remain_mm = clampf(remaining * mm_per_t, 0.0f, 1e6f);
  if (remain_mm < END_STOP_L) {
    pwmL = 0; pwmR = 0;
  }

  // --- Command motors (your ordering is motor.setPWM(right, left)) ---
  left_pwm = pwmL;
  right_pwm = pwmR;
  v_cmd_prev = v_cmd;

  // --- LOGGING (CSV) ---
  // Serial.print(millis()); Serial.print(',');
  // Serial.print(xr,1); Serial.print(','); Serial.print(yr,1); Serial.print(','); Serial.print(hr,4); Serial.print(',');
  // Serial.print(t_hat,4); Serial.print(',');
  // Serial.print(x_ref,1); Serial.print(','); Serial.print(y_ref,1); Serial.print(',');
  // Serial.print(x_look,1); Serial.print(','); Serial.print(y_look,1); Serial.print(',');
  // Serial.print(v_cmd,1); Serial.print(','); Serial.print(omega_cmd,4); Serial.print(','); Serial.print(kappa_cmd,5); Serial.print(',');
  Serial.print(vL_des,2); Serial.print(','); 
  Serial.print(vL_meas,2); Serial.print(" | ");
  Serial.print(vR_des,2); Serial.print(','); Serial.print(vR_meas,2); 
  Serial.println(" ");
  // Serial.print(pwmL); Serial.print(','); Serial.print(pwmR); Serial.print(',');
  // Serial.println(frontRange,1);
}

// // ================== SPLINE BUILD & EVAL ==================
// // Build chord-length parameter and natural cubic (C^2) for X(t), Y(t)
// void buildNaturalCubic() {
//   // chord-length parameterization
//   T_param[0] = 0.0f;
//   total_chord = 0.0f;
//   for (int i = 1; i < N_WP; ++i) {
//     float dx = WP_X[i] - WP_X[i-1];
//     float dy = WP_Y[i] - WP_Y[i-1];
//     total_chord += sqrtf(dx*dx + dy*dy);
//     T_param[i] = total_chord;
//   }
//   if (total_chord < 1e-6f) total_chord = 1.0f; // avoid div0

//   // Solve for second derivatives Y2 and X2 (Natural spline: second deriv = 0 at ends)
//   auto solveY2 = [&](const float* Y, float* Y2) {
//     static float u[MAX_N];
//     Y2[0] = 0.0f; u[0] = 0.0f;
//     for (int i = 1; i < N_WP - 1; ++i) {
//       float h_i   = T_param[i]   - T_param[i-1];
//       float h_ip1 = T_param[i+1] - T_param[i];
//       float sig = h_i / (h_i + h_ip1);
//       float p = sig * Y2[i-1] + 2.0f;
//       Y2[i] = (sig - 1.0f) / p;
//       float dY  = (Y[i+1] - Y[i]) / h_ip1 - (Y[i] - Y[i-1]) / h_i;
//       u[i] = (6.0f * dY / (h_i + h_ip1) - sig * u[i-1]) / p;
//     }
//     Y2[N_WP-1] = 0.0f;
//     for (int k = N_WP - 2; k >= 0; --k) {
//       Y2[k] = Y2[k] * Y2[k+1] + u[k];
//       Serial.print(Y2[k]);
//       Serial.print(", ");
//     }
//     Serial.println(".");
//   };

//   solveY2(WP_X, X2);
//   solveY2(WP_Y, Y2);
// }
// // Evaluate S(t) and its first/second derivatives using second-derivatives form
// void splineEval(float tq, float& x, float& y, float& dx, float& dy, float& ddx, float& ddy) {
//   // Clamp in range
//   tq = clampf(tq, T_param[0], T_param[N_WP-1]);

//   // Find interval i such that tq in [Ti, Ti+1]
//   int i = 0;
//   int hi = N_WP - 1;
//   while (hi - i > 1) {
//     int mid = (i + hi) >> 1;
//     if (T_param[mid] > tq) hi = mid;
//     else i = mid;
//   }
//   float h = T_param[i+1] - T_param[i];
//   if (h < 1e-9f) h = 1e-9f;

//   float A = (T_param[i+1] - tq) / h;
//   float B = (tq - T_param[i]) / h;

//   // Position
//   x = A*WP_X[i] + B*WP_X[i+1] + ((A*A*A - A)*X2[i] + (B*B*B - B)*X2[i+1]) * (h*h)/6.0f;
//   y = A*WP_Y[i] + B*WP_Y[i+1] + ((A*A*A - A)*Y2[i] + (B*B*B - B)*Y2[i+1]) * (h*h)/6.0f;

//   // First derivative dS/dt
//   dx = (WP_X[i+1] - WP_X[i])/h + ( (-3*A*A + 1)*X2[i] + (3*B*B - 1)*X2[i+1] ) * h / 6.0f;
//   dy = (WP_Y[i+1] - WP_Y[i])/h + ( (-3*A*A + 1)*Y2[i] + (3*B*B - 1)*Y2[i+1] ) * h / 6.0f;

//   // Second derivative d²S/dt²
//   ddx = A*X2[i] + B*X2[i+1];
//   ddy = A*Y2[i] + B*Y2[i+1];
// }

void buildCatmullRom() {
  if (N_WP < 2) return;

  // (a) centripetal parameter: t_{i+1} = t_i + |Pi+1-Pi|^0.5
  tK[0] = 0.0f;
  for (int i = 1; i < N_WP; ++i) {
    float d = hypotf(WP_X[i]-WP_X[i-1], WP_Y[i]-WP_Y[i-1]);  // |ΔP|
    tK[i] = tK[i-1] + sqrtf(fmaxf(d, 1e-6f));
  }

  // (b) end tangents (one-sided, w.r.t. tK)
  Tx[0]       = (WP_X[1]      - WP_X[0])      / (tK[1]       - tK[0]);
  Ty[0]       = (WP_Y[1]      - WP_Y[0])      / (tK[1]       - tK[0]);
  Tx[N_WP-1]  = (WP_X[N_WP-1] - WP_X[N_WP-2]) / (tK[N_WP-1]  - tK[N_WP-2]);
  Ty[N_WP-1]  = (WP_Y[N_WP-1] - WP_Y[N_WP-2]) / (tK[N_WP-1]  - tK[N_WP-2]);

  // (c) interior tangents (non-uniform CR: weighted average of neighbor slopes)
  for (int i = 1; i < N_WP-1; ++i) {
    float dt0 = tK[i]   - tK[i-1];
    float dt1 = tK[i+1] - tK[i];
    float sx0 = (WP_X[i]   - WP_X[i-1]) / dt0;   // left slope
    float sx1 = (WP_X[i+1] - WP_X[i])   / dt1;   // right slope
    float sy0 = (WP_Y[i]   - WP_Y[i-1]) / dt0;
    float sy1 = (WP_Y[i+1] - WP_Y[i])   / dt1;
    float w0 = dt1 / (dt0 + dt1);
    float w1 = dt0 / (dt0 + dt1);
    float w  = (1.0f - Tension);
    Tx[i] = w * (sx0 * w0 + sx1 * w1);
    Ty[i] = w * (sy0 * w0 + sy1 * w1);
  }

  // (d) build chord-length (mm) parameter for your tracker
  T_param[0] = 0.0f;
  for (int i = 1; i < N_WP; ++i) {
    T_param[i] = T_param[i-1] + hypotf(WP_X[i]-WP_X[i-1], WP_Y[i]-WP_Y[i-1]);
  }
}

// Hermite basis evaluation on segment i (t in [tK[i], tK[i+1]])
static void evalCRSeg(int i, float t, float& x, float& y, float& dx, float& dy, float& ddx, float& ddy) {
  float t0 = tK[i], t1 = tK[i+1], h = t1 - t0;
  if (h < 1e-9f) h = 1e-9f;
  float u = (t - t0) / h;              // 0..1

  // Hermite basis
  float h00 = (2*u*u*u - 3*u*u + 1);
  float h10 = (u*u*u - 2*u*u + u);
  float h01 = (-2*u*u*u + 3*u*u);
  float h11 = (u*u*u - u*u);

  float px0 = WP_X[i], px1 = WP_X[i+1];
  float py0 = WP_Y[i], py1 = WP_Y[i+1];
  float mx0 = Tx[i]*h, mx1 = Tx[i+1]*h;   // scale tangents by segment length
  float my0 = Ty[i]*h, my1 = Ty[i+1]*h;

  x = h00*px0 + h10*mx0 + h01*px1 + h11*mx1;
  y = h00*py0 + h10*my0 + h01*py1 + h11*my1;

  // First derivative wrt tK (needed for curvature and advanceByArc)
  float dh00 = (6*u*u - 6*u) / h;
  float dh10 = (3*u*u - 4*u + 1) / h;
  float dh01 = (-6*u*u + 6*u) / h;
  float dh11 = (3*u*u - 2*u) / h;

  dx = dh00*px0 + dh10*mx0 + dh01*px1 + dh11*mx1;
  dy = dh00*py0 + dh10*my0 + dh01*py1 + dh11*my1;

  // Second derivative (optional; rough form for curvature)
  float d2h00 = (12*u - 6) / (h*h);
  float d2h10 = (6*u - 4) / (h*h);
  float d2h01 = (-12*u + 6) / (h*h);
  float d2h11 = (6*u - 2) / (h*h);
  ddx = d2h00*px0 + d2h10*mx0 + d2h01*px1 + d2h11*mx1;
  ddy = d2h00*py0 + d2h10*my0 + d2h01*py1 + d2h11*my1;
}

// Wrapper like your splineEval (uses t in the same overall range)
void splineEval(float t_arc, float& x, float& y,
                float& dx, float& dy, float& ddx, float& ddy) {
  // Clamp in chord-length (mm) parameter space
  if (t_arc <= T_param[0]) t_arc = T_param[0];
  if (t_arc >= T_param[N_WP-1]) t_arc = T_param[N_WP-1] - 1e-6f;

  // Find segment j such that t_arc ∈ [T_param[j], T_param[j+1]]
  int j = 0, hi = N_WP - 1;
  while (hi - j > 1) {
    int m = (j + hi) >> 1;
    if (T_param[m] > t_arc) hi = m; else j = m;
  }

  // Fraction in chord space
  float hArc = T_param[j+1] - T_param[j];
  if (hArc < 1e-9f) hArc = 1e-9f;
  float u = (t_arc - T_param[j]) / hArc;       // 0..1

  // Map that fraction to Catmull–Rom param inside the same segment
  float t0K = tK[j];
  float hK  = tK[j+1] - tK[j];
  if (hK < 1e-9f) hK = 1e-9f;
  float t_cr = t0K + u * hK;

  // Evaluate CR Hermite on segment j at t_cr (returns derivs w.r.t. tK)
  float dxK, dyK, ddxK, ddyK;
  evalCRSeg(j, t_cr, x, y, dxK, dyK, ddxK, ddyK);

  // Chain rule: convert derivatives to be w.r.t. chord-length (mm) parameter
  float s = hK / hArc;              // dtK / d(mm-param) on this segment
  dx  = dxK  * s;
  dy  = dyK  * s;
  ddx = ddxK * s * s;
  ddy = ddyK * s * s;
}


