// path_waypoints_simple.h
#pragma once
#include <vector>
#include <cmath>
#include <algorithm>

struct Cell { int r, c; };
struct Pt   { float x, y; };

// --- tiny helpers ---
static inline Pt operator+(const Pt& a, const Pt& b){ return {a.x+b.x, a.y+b.y}; }
static inline Pt operator-(const Pt& a, const Pt& b){ return {a.x-b.x, a.y-b.y}; }
static inline Pt operator*(const Pt& a, float s)    { return {a.x*s, a.y*s}; }
static inline float dot(const Pt& a, const Pt& b){ return a.x*b.x + a.y*b.y; }
static inline float cross(const Pt& a, const Pt& b){ return a.x*b.y - a.y*b.x; }
static inline float norm(const Pt& a){ return std::sqrt(a.x*a.x + a.y*a.y); }
static inline Pt   unit(const Pt& a){ float n=norm(a); return (n>1e-9f)? Pt{a.x/n,a.y/n} : Pt{0,0}; }
static inline float clampf(float v,float lo,float hi){ return v<lo?lo:(v>hi?hi:v); }

static inline Pt cellCenter(const Cell& k, float cell_mm){
  return Pt{ (k.c + 0.5f)*cell_mm, (k.r + 0.5f)*cell_mm };
}

static inline Pt clampToMazeBox(Pt p, float cell_mm, int cols, int rows, float clearance){
  const float W = cols * cell_mm, H = rows * cell_mm;
  p.x = clampf(p.x, clearance, W - clearance);
  p.y = clampf(p.y, clearance, H - clearance);
  return p;
}

// push points every step_mm from A to B (including B), assuming straight line
static void pushStraight(std::vector<Pt>& out, Pt A, Pt B, float step_mm){
  Pt v = B - A; float L = norm(v);
  if (L < 1e-6f){ if (out.empty() || norm(B-out.back())>1e-3f) out.push_back(B); return; }
  Pt u = v*(1.0f/L);
  float s = 0.0f;
  if (out.empty() || norm(A - out.back()) > 1e-3f) out.push_back(A);
  while (s + step_mm < L) { s += step_mm; out.push_back(A + u*s); }
  out.push_back(B);
}

// add circular arc (Tin→Tout) with center C and direction (ccw if ccw>0), points every step_mm
static void pushArc(std::vector<Pt>& out, Pt Tin, Pt Tout, Pt C, bool ccw, float step_mm){
  Pt r0 = Tin - C, r1 = Tout - C;
  float R = norm(r0);
  if (R < 1e-6f){ out.push_back(Tout); return; }

  auto ang = [](float y, float x){ return std::atan2(y, x); };
  float a0 = ang(r0.y, r0.x);
  float a1 = ang(r1.y, r1.x);

  // normalize sweep direction
  float d = a1 - a0;
  if (ccw) { while (d <= 0) d += 2*M_PI; }
  else     { while (d >= 0) d -= 2*M_PI; d = -d; } // use positive sweep length

  int N = std::max(1, (int)std::ceil((R * d) / step_mm));
  for (int i = 1; i <= N; ++i) {
    float t = (float)i / (float)N;
    float a = ccw ? (a0 + t*d) : (a0 - t*d);
    out.push_back( Pt{ C.x + R*std::cos(a), C.y + R*std::sin(a) } );
  }
}

// --- main API ---
// cells: ordered list of grid cells you’ll traverse
// maze_cols/rows: maze size in cells
// cell_mm: cell size
// clearance_mm: distance from outer walls you must maintain
// radius_req_mm: desired corner radius (exact when feasible; clipped if too big)
// step_mm: spacing between emitted waypoints on both straights and arcs
inline std::vector<Pt> generateWaypointsFixedRadius(
  const std::vector<Cell>& cells,
  int maze_cols, int maze_rows,
  float cell_mm,
  float clearance_mm,
  float radius_req_mm = 90.0f,
  float step_mm       = 50.0f
){
  std::vector<Pt> out;
  if (cells.empty()) return out;

  // Precompute centerline points, clamped to outer box for clearance from outer walls
  std::vector<Pt> ctr;
  ctr.reserve(cells.size());
  for (auto& c : cells) {
    ctr.push_back(clampToMazeBox(cellCenter(c, cell_mm), cell_mm, maze_cols, maze_rows, clearance_mm));
  }

  // Early exit: 1 point
  if (ctr.size() == 1) { out.push_back(ctr[0]); return out; }
  
  const float R_corner = radius_req_mm;

  // Start at first center
  out.push_back(ctr[0]);

  for (size_t i = 1; i + 1 < ctr.size(); ++i) {
    Pt P0 = ctr[i-1];
    Pt P1 = ctr[i];     // turning cell center
    Pt P2 = ctr[i+1];

    Pt a = unit(P1 - P0);  // incoming dir
    Pt b = unit(P2 - P1);  // outgoing dir
    if (norm(a) < 1e-6f || norm(b) < 1e-6f) { // degenerate
      pushStraight(out, out.back(), P1, step_mm);
      continue;
    }

    float col = std::fabs(dot(a, b));
    if (col > 0.999f) {
        // Straight section
        pushStraight(out, out.back(), P1, step_mm);
        continue;
    }

    // --- Circle-based 90° arc ---
    float d = R_corner; // tangent offset

    // Tangent points along each leg
    Pt Tin  = P1 - a * d;
    Pt Tout = P1 + b * d;

    // Arc center for 90° turn
    Pt u_in  = unit(P0 - P1);   // opposite of a
    Pt u_out = unit(P2 - P1);   // same as b
    Pt u_bis = unit(u_in + u_out);

    // For 90°, center distance = R / sin(45°) = R * sqrt(2)
    float center_dist = R_corner * std::sqrt(2.0f);
    Pt C = P1 + u_bis * center_dist;

    // Decide arc direction
    bool ccw = (cross(a, b) > 0);

    // --- Generate arc points using circle equation ---
    Pt startVec = Tin - C;  // vector from center to start point
    Pt endVec   = Tout - C;

    float startAng = atan2f(startVec.y, startVec.x);
    float endAng   = atan2f(endVec.y, endVec.x);

    // Ensure CCW or CW ordering
    if (ccw && endAng < startAng) endAng += M_PI_2; // 90°
    if (!ccw && startAng < endAng) startAng += M_PI_2;

    int numSteps = std::ceil((M_PI_2 * R_corner) / (step_mm) * 5) ; // quarter circle length / step
    float angStep = (ccw ? 1 : -1) * (M_PI_2 / numSteps);

    pushStraight(out, out.back(), Tin, step_mm);

    for (int i = 1; i < numSteps; ++i) {
        float theta = startAng + i * angStep;
        Pt p = { C.x + R_corner * cosf(theta), C.y + R_corner * sinf(theta) };
        out.push_back(p);
    }

    pushStraight(out, out.back(), Tout, step_mm);
  }


  // Final straight to last center
  pushStraight(out, out.back(), ctr.back(), step_mm);

  // Safety: clamp waypoints to outer box (should already be inside)
  for (auto& p : out) p = clampToMazeBox(p, cell_mm, maze_cols, maze_rows, clearance_mm);

  // Optional dedupe
  std::vector<Pt> clean; clean.reserve(out.size());
  const float MIN_DS = 1.0f;
  for (auto& p : out) {
    if (clean.empty() || norm(p - clean.back()) > MIN_DS) clean.push_back(p);
  }
  return clean;
}

// --- main API ---
// Places waypoints every `step_mm` on straights.
// For each 90° corner at cell P1 (between P0→P1→P2), inserts the two
// mid-edge points: Ein = (P0+P1)/2 and Eout = (P1+P2)/2, producing two 45° turns.
inline std::vector<Pt> generateWaypointsChamfer90(
  const std::vector<Cell>& cells,
  int maze_cols, int maze_rows,
  float cell_mm,
  float clearance_mm,
  float step_mm = 50.0f
){
  std::vector<Pt> out;
  if (cells.empty()) return out;

  // Centerline (clamped to outer box)
  std::vector<Pt> ctr; ctr.reserve(cells.size());
  for (auto& c : cells)
    ctr.push_back(clampToMazeBox(cellCenter(c, cell_mm), cell_mm, maze_cols, maze_rows, clearance_mm));

  if (ctr.size() == 1) { out.push_back(ctr[0]); return out; }

  // Start from the first center
  out.push_back(ctr[0]);

  for (size_t i = 1; i + 1 < ctr.size(); ++i) {
    Pt P0 = ctr[i-1];
    Pt P1 = ctr[i];
    Pt P2 = ctr[i+1];

    Pt a = unit(P1 - P0);            // incoming dir
    Pt b = unit(P2 - P1);            // outgoing dir

    // Degenerate or missing direction → just march to P1
    if (norm(a) < 1e-6f || norm(b) < 1e-6f) {
      pushStraight(out, out.back(), P1, step_mm);
      continue;
    }

    float col = std::fabs(dot(a, b));

    if (col > 0.999f) {
      // Straight-through
      pushStraight(out, out.back(), P1, step_mm);
      continue;
    }

    // --- 90° corner: insert edge midpoints of the turning cell ---
    // Edge midpoint on entry side (between P0 and P1)
    Pt Ein  = Pt{ 0.5f*(P0.x + P1.x), 0.5f*(P0.y + P1.y) };
    // Edge midpoint on exit side (between P1 and P2)
    Pt Eout = Pt{ 0.5f*(P1.x + P2.x), 0.5f*(P1.y + P2.y) };

    // Clamp to keep clearance from outer boundary (inner walls are handled by
    // using centerlines and edge midpoints)
    Ein  = clampToMazeBox(Ein,  cell_mm, maze_cols, maze_rows, clearance_mm);
    Eout = clampToMazeBox(Eout, cell_mm, maze_cols, maze_rows, clearance_mm);

    // Go: ... → Ein (still straight along the incoming corridor)
    pushStraight(out, out.back(), Ein, step_mm);
    // Then a single diagonal segment Ein → Eout (this is the 45° “chamfer”)
    pushStraight(out, Ein, Eout, step_mm);
    // We do NOT push P1 itself (avoids the tight V/loop); continue to next
  }

  // Finish with the last center
  pushStraight(out, out.back(), ctr.back(), step_mm);

  // Light dedupe
  std::vector<Pt> clean; clean.reserve(out.size());
  const float MIN_DS = 1.0f;
  for (auto& p : out) {
    if (clean.empty() || norm(p - clean.back()) > MIN_DS) clean.push_back(p);
  }
  return clean;
}

inline std::vector<Pt> generateWaypointsCenters(
  const std::vector<Cell>& cells,
  int maze_cols, int maze_rows,
  float cell_mm,
  float clearance_mm
){
  std::vector<Pt> out;
  out.reserve(cells.size());

  for (auto& c : cells) {
    Pt p = cellCenter(c, cell_mm);
    // keep inside the maze bounds with clearance from walls
    p = clampToMazeBox(p, cell_mm, maze_cols, maze_rows, clearance_mm);
    out.push_back(p);
  }

  return out;
}

