#include <emscripten/emscripten.h>

#include <cmath>

#include "controller.h"
#include "map.h"
#include "planner.h"
#include "quad.h"
#include "route.h"

#include <vector>

// the one drone the page flies, its flight controller and where it's told to go
static QuadParams P;
static QuadState s, prev;
static Controller c;
static Setpoint sp;

// the physics step, same 1 ms the controller was tuned on
static const double DT = 0.001;

// the city, its planning grid, and the path being flown with how long it's been flying it
static std::vector<Box> boxes, grown;
static Grid grid(1, 1, 1);
static std::vector<Vec3> path;
static double path_t = 0, path_yaw = 0;
static bool following = false;

// path speed: 5 m/s cruise with 3 m/s^2 ramps, stopping at each waypoint (the ramped route from results/control.md)
static const double CRUISE = 5, ACCEL = 3;

// buildings as lo x y z, hi x y z (6 numbers each) and the path as x y z, for the page to draw.
// a city has at most 24 buildings, 5 x 5 lots minus home
static double box_buf[6 * 24], path_buf[3 * 256];

// the last two states packed as p, v, q (w x y z), body rates, rotor speeds, 17 numbers each
// the page draws the drone part way between them
static double now_buf[17], prev_buf[17];

// copies a state into one of the buffers above
static void pack(const QuadState& st, double* out) {
  const double v[17] = {st.p.x, st.p.y, st.p.z, st.v.x, st.v.y, st.v.z, st.q.w, st.q.x, st.q.y, st.q.z,
                        st.w.x, st.w.y, st.w.z, st.rotor[0], st.rotor[1], st.rotor[2], st.rotor[3]};
  for (int i = 0; i < 17; i++) out[i] = v[i];
}

extern "C" {

// puts the drone hovering at a spot facing yaw, with the rotors already at hover speed and a fresh controller
EMSCRIPTEN_KEEPALIVE void reset(double x, double y, double z, double yaw) {
  s = QuadState{};
  s.p = {x, y, z};
  s.q = axis_angle({0, 0, 1}, yaw);
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  s.rotor = {w, w, w, w};
  c = Controller{};
  sp = Setpoint{};
  following = false;
  sp.pos = s.p;
  sp.yaw = yaw;
  prev = s;
  pack(s, now_buf);
  pack(prev, prev_buf);
}

// tells the controller where to fly and which way to face, it holds there once it arrives
EMSCRIPTEN_KEEPALIVE void set_target(double x, double y, double z, double yaw) {
  following = false;
  sp = Setpoint{};
  sp.pos = {x, y, z};
  sp.yaw = yaw;
}

// runs the controller and physics forward some 1 ms steps. the page calls it one step at a time so it can check
// the mission after each step, a big batch is only used to time the pure wasm cost
EMSCRIPTEN_KEEPALIVE void advance(int steps) {
  for (int i = 0; i < steps; i++) {
    // when flying a path, the setpoint slides along it
    if (following) {
      path_t += DT;
      sp = follow(path, path_t, CRUISE, ACCEL, path_yaw);
    }
    prev = s;
    s = step(P, s, control(c, P, s, sp, DT), DT);
  }
  pack(s, now_buf);
  pack(prev, prev_buf);
}

// where the two state buffers sit in wasm memory, so js can read them in place
EMSCRIPTEN_KEEPALIVE double* state_now() { return now_buf; }
EMSCRIPTEN_KEEPALIVE double* state_prev() { return prev_buf; }

// where rotor i sits on the body, so the 3d model matches the physics instead of copying numbers
EMSCRIPTEN_KEEPALIVE double rotor_pos(int i, int axis) {
  const Vec3& r = P.rotor_pos[i];
  return axis == 0 ? r.x : axis == 1 ? r.y : r.z;
}

// builds the seeded city and its planning grid, returns how many buildings it has
EMSCRIPTEN_KEEPALIVE int new_city(unsigned seed) {
  boxes = city(seed);
  grid = city_grid(boxes);
  grown.clear();
  for (size_t i = 0; i < boxes.size(); i++) {
    const Box& b = boxes[i];
    const double v[6] = {b.lo.x, b.lo.y, b.lo.z, b.hi.x, b.hi.y, b.hi.z};
    for (int k = 0; k < 6; k++) box_buf[6 * i + k] = v[k];
    grown.push_back(grow(b, kMargin));
  }
  return int(boxes.size());
}
EMSCRIPTEN_KEEPALIVE double* boxes_ptr() { return box_buf; }

// plans a smoothed path from the drone's spot to the goal and starts flying it.
// returns how many waypoints it has, 0 when there's no way there (the drone then just holds)
EMSCRIPTEN_KEEPALIVE int fly_to(double x, double y, double z) {
  path = smooth(astar(grid, s.p, {x, y, z}), grown);
  if (path.size() > 256) path.clear();
  for (size_t i = 0; i < path.size(); i++) {
    path_buf[3 * i] = path[i].x;
    path_buf[3 * i + 1] = path[i].y;
    path_buf[3 * i + 2] = path[i].z;
  }
  // keep facing the way the drone faces now until the path moves sideways
  Vec3 nose = rotate(s.q, {1, 0, 0});
  path_yaw = std::atan2(nose.y, nose.x);
  path_t = 0;
  following = !path.empty();
  return int(path.size());
}
EMSCRIPTEN_KEEPALIVE double* path_ptr() { return path_buf; }

// true once the setpoint has reached the end of the path
EMSCRIPTEN_KEEPALIVE int path_done() { return !following || path_t >= path_time(path, CRUISE, ACCEL); }

// the physics step in seconds, so the page's clock uses the same one
EMSCRIPTEN_KEEPALIVE double step_size() { return DT; }
}
