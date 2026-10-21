#include <emscripten/emscripten.h>

#include <algorithm>
#include <cmath>

#include "controller.h"
#include "estimator.h"
#include "lidar.h"
#include "map.h"
#include "planner.h"
#include "quad.h"
#include "route.h"
#include "wind.h"

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

// the sensors, the estimate the controller flies on (rtk gps, same as results/lidar.md) and the lidar.
// tick counts physics steps so gps, magnetometer and lidar each run every 100 steps
static Sensors sn;
static Estimator est;
static QuadState e, e_prev;
static Lidar L;
static long tick = 0;
static const unsigned SENSOR_SEED = 1;

// the real world (the city plus boxes the drone hasn't been told about), where it's headed and how often it replanned
static std::vector<Box> world;
static Vec3 goal;
static int replans = 0;

// the wind (breeze from the west plus gusts) and the wind the drone feels right now as x y z.
// a crash freezes the sim until the next reset. truth makes the controller and planner use the real state
static Wind wnd = make_wind({}, 0, 2);
static double wind_buf[3];
static bool crash = false, truth = false;

// the state everything flies on: the estimate, or the real state in truth mode
static const QuadState& nav() { return truth ? s : e; }

// the last lidar turn's hit points as x y z, 16 beams x 180 steps at most
static double hit_buf[3 * 16 * 180];
static int hit_count = 0;

// the last two states packed as p, v, q (w x y z), body rates, rotor speeds, 17 numbers each
// the page draws the drone part way between them, and the ghost between the last two estimates
static double now_buf[17], prev_buf[17], est_buf[17], est_prev_buf[17];

// copies the path into the buffer the page draws from. a path too long for it gets dropped
static void publish_path() {
  if (path.size() > 256) path.clear();
  for (size_t i = 0; i < path.size(); i++) {
    path_buf[3 * i] = path[i].x;
    path_buf[3 * i + 1] = path[i].y;
    path_buf[3 * i + 2] = path[i].z;
  }
}

// one lidar turn: keeps the hits for the page, adds new obstacles to the map and replans when they cut the path.
// the same rule as tools/lidar_report.cpp, both call replan() in core
static void lidar_turn() {
  std::vector<Vec3> hits = scan(L, sn, s, nav(), world);
  hit_count = int(std::min(hits.size(), size_t(16 * 180)));
  for (int i = 0; i < hit_count; i++) {
    hit_buf[3 * i] = hits[i].x;
    hit_buf[3 * i + 1] = hits[i].y;
    hit_buf[3 * i + 2] = hits[i].z;
  }
  size_t known = grown.size();
  const QuadState& n = nav();
  if (add_hits(grid, grown, hits, boxes, n.p) && following && replan(path, grid, grown, known, n.p, goal)) {
    // no way through, or a path too long to draw: hold where it thinks it is.
    // the hold is one point, so nothing can cut it and it won't replan again on its own
    publish_path();
    if (path.empty()) {
      path = {n.p};
      publish_path();
    }
    path_yaw = heading(n.q);
    path_t = 0;
    replans++;
  }
}

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

  // fresh rtk sensors and estimator, the gyro calibrated for 1 s on the pad like the native flights
  Noise n = rtk();
  sn = make_sensors(n, SENSOR_SEED);
  est = make_estimator(s, n);
  Vec3 gsum;
  for (int i = 0; i < 1000; i++) gsum = gsum + read_imu(sn, P, s, DT).gyro;
  est.gyro_bias = gsum * 0.001;
  e = e_prev = estimate(est);
  pack(e, est_buf);
  pack(e, est_prev_buf);
  tick = 0;
  hit_count = 0;
  crash = false;
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
  for (int i = 0; i < steps && !crash; i++) {
    tick++;
    // imu every step, gps and magnetometer at 10 Hz
    predict(est, read_imu(sn, P, s, DT), DT);
    if (tick % 100 == 0) {
      gps(est, read_gps(sn, s));
      mag(est, read_mag(sn, s));
    }
    e_prev = e;
    e = estimate(est);

    // lidar at 10 Hz, half a period off the gps. a replan restarts the path clock, otherwise it ticks on
    int before = replans;
    if (tick % 100 == 50) lidar_turn();
    if (following && replans == before) path_t += DT;

    // when flying a path, the setpoint slides along it
    if (following) sp = follow(path, path_t, CRUISE, ACCEL, path_yaw);

    // the controller flies on the estimate (or the truth), the physics moves the real drone through the wind
    prev = s;
    Vec3 air = wind_step(wnd, DT);
    s = step(P, s, control(c, P, nav(), sp, DT), DT, air);
    wind_buf[0] = air.x, wind_buf[1] = air.y, wind_buf[2] = air.z;
    // touching a box or the ground ends the run
    if (crashed(s.p, world)) crash = true;
  }
  pack(s, now_buf);
  pack(prev, prev_buf);
  pack(e, est_buf);
  pack(e_prev, est_prev_buf);
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
  world = boxes;
  hit_count = 0;
  replans = 0;
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

// puts a box in the real world that the drone isn't told about, only the lidar can find it.
// a box that would land on the drone isn't added, returns 0 then
EMSCRIPTEN_KEEPALIVE int add_box(double x0, double y0, double z0, double x1, double y1, double z1) {
  Box b{{x0, y0, z0}, {x1, y1, z1}};
  if (crashed(s.p, {b}) && s.p.z >= kReach) return 0;
  world.push_back(b);
  return 1;
}

// breeze from the west in m/s and gust strength in m/s, both from the panel sliders
EMSCRIPTEN_KEEPALIVE void set_wind(double breeze, double gusts) {
  wnd.breeze = {breeze, 0, 0};
  wnd.sigma = gusts;
  if (gusts == 0) wnd.gust = {};
}
EMSCRIPTEN_KEEPALIVE double* wind_now() { return wind_buf; }

// gps noise toggle: 1 is the everyday neo-m8n (meters off), 0 the rtk one (about a centimeter).
// only the gps differs between the two, so the sensors and the filter swap just those numbers
EMSCRIPTEN_KEEPALIVE void set_noisy_gps(int on) {
  Noise n = on ? Noise{} : rtk();
  sn.n.gps_xy = est.n.gps_xy = n.gps_xy;
  sn.n.gps_z = est.n.gps_z = n.gps_z;
}

// truth toggle: 1 flies and plans on the real state, 0 on the estimate
EMSCRIPTEN_KEEPALIVE void set_truth(int on) { truth = on; }

// 1 once the drone has touched a box or the ground
EMSCRIPTEN_KEEPALIVE int is_crashed() { return crash; }

// plans a smoothed path from where the drone thinks it is to the goal and starts flying it.
// returns how many waypoints it has, 0 when there's no way there (the drone then just holds)
EMSCRIPTEN_KEEPALIVE int fly_to(double x, double y, double z) {
  goal = {x, y, z};
  path = smooth(astar(grid, nav().p, goal), grown);
  publish_path();
  // keep facing the way it thinks it faces now until the path moves sideways
  path_yaw = heading(nav().q);
  path_t = 0;
  following = !path.empty();
  return int(path.size());
}
EMSCRIPTEN_KEEPALIVE double* path_ptr() { return path_buf; }

// the path after a replan: how many waypoints, and how many replans so far so the page knows to redraw
EMSCRIPTEN_KEEPALIVE int path_len() { return int(path.size()); }
EMSCRIPTEN_KEEPALIVE int replan_count() { return replans; }

// physics steps since reset, so the page times its plot and lidar dots off the same clock as the sensors
EMSCRIPTEN_KEEPALIVE double sim_tick() { return double(tick); }

// the estimate in the same 17 number layout as the state, now and one step back
EMSCRIPTEN_KEEPALIVE double* est_now() { return est_buf; }
EMSCRIPTEN_KEEPALIVE double* est_prev() { return est_prev_buf; }

// the last lidar turn's hits, x y z each
EMSCRIPTEN_KEEPALIVE double* hits_ptr() { return hit_buf; }
EMSCRIPTEN_KEEPALIVE int hits_len() { return hit_count; }

// true once the setpoint has reached the end of the path
EMSCRIPTEN_KEEPALIVE int path_done() { return !following || path_t >= path_time(path, CRUISE, ACCEL); }

// the physics step in seconds, so the page's clock uses the same one
EMSCRIPTEN_KEEPALIVE double step_size() { return DT; }
}
