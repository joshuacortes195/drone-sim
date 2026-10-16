#include "route.h"
#include "step_response.h"

#include <chrono>
#include <cstdio>
#include <vector>

// the fixed test route: out east, climb while going north, back west, then home and down
static const std::vector<Vec3> kRoute = {{0, 0, 10}, {20, 0, 10}, {20, 20, 15}, {0, 20, 15}, {0, 0, 10}};

// closest distance from point p to the segment a-b
static double to_segment(Vec3 p, Vec3 a, Vec3 b) {
  Vec3 ab = b - a;
  double t = std::clamp(dot(p - a, ab) / dot(ab, ab), 0.0, 1.0);
  return norm(p - (a + ab * t));
}

// how one run along the route went
struct RouteInfo {
  double rms_ref, max_ref, rms_cross, seconds;
};

// slides a reference point along the route and has the drone chase it, with the point's velocity and
// acceleration fed forward, scoring distance to that point and to the route line every 1 ms step
static RouteInfo fly_route(double cruise, double accel) {
  QuadParams P;
  Controller c;
  QuadState s;
  s.p = kRoute[0];
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  s.rotor = {w, w, w, w};
  double sum_ref = 0, sum_cross = 0, max_ref = 0;
  long n = 0;
  for (size_t k = 0; k + 1 < kRoute.size(); k++) {
    Vec3 a = kRoute[k], b = kRoute[k + 1];
    Vec3 dir = (b - a) * (1 / norm(b - a));
    int steps = (int)std::lround(leg_time(norm(b - a), cruise, accel) / 0.001);
    for (int i = 1; i <= steps; i++) {
      // where the reference is now on this leg, and how fast it moves
      double dist, speed, acc;
      along(norm(b - a), cruise, accel, i * 0.001, dist, speed, acc);
      Setpoint sp;
      sp.pos = a + dir * dist;
      sp.vel = dir * speed;
      sp.acc = dir * acc;
      s = step(P, s, control(c, P, s, sp, 0.001), 0.001);
      // error to the moving point, and to the nearest part of the whole route
      double e = norm(s.p - sp.pos);
      double cross = 1e9;
      for (size_t j = 0; j + 1 < kRoute.size(); j++) cross = std::min(cross, to_segment(s.p, kRoute[j], kRoute[j + 1]));
      sum_ref += e * e;
      sum_cross += cross * cross;
      max_ref = std::max(max_ref, e);
      n++;
    }
  }
  return {std::sqrt(sum_ref / n), max_ref, std::sqrt(sum_cross / n), n * 0.001};
}

// starts hovering at 10 m tipped by `angle` about x and flies 10 s holding (0, 0, 10). prints the worst
// distance from the set point, the most height lost, when it first got back under 5 degrees of tilt, and
// how far off it is at 6 s and 10 s
static void recovery(double angle) {
  QuadParams P;
  Controller c;
  Setpoint sp;
  sp.pos = {0, 0, 10};
  QuadState s;
  s.p = sp.pos;
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  s.rotor = {w, w, w, w};
  s.q = axis_angle({1, 0, 0}, angle);
  double worst = 0, lowest = 10, level_at = -1, at6 = 0;
  for (int i = 1; i <= 10000; i++) {
    s = step(P, s, control(c, P, s, sp, 0.001), 0.001);
    worst = std::max(worst, norm(s.p - sp.pos));
    lowest = std::min(lowest, s.p.z);
    if (level_at < 0 && rotate(s.q, {0, 0, 1}).z > std::cos(0.0873)) level_at = i * 0.001;
    if (i == 6000) at6 = norm(s.p - sp.pos);
  }
  std::printf("  %3.0f deg  worst %.3f m  height lost %.3f m  under 5 deg tilt at %.2f s  off by %.4f m at 6 s, %.4f m at 10 s\n",
              angle * 57.2958, worst, 10 - lowest, level_at, at6, norm(s.p - sp.pos));
}

// a plain 10 m set point jump in x with no path, where the speed and tilt limits and the integral all matter
static void big_step() {
  QuadParams P;
  Controller c;
  Setpoint sp;
  sp.pos = {10, 0, 10};
  QuadState s;
  s.p = {0, 0, 10};
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  s.rotor = {w, w, w, w};
  double past = 0;
  std::printf("  ");
  for (int i = 1; i <= 15000; i++) {
    s = step(P, s, control(c, P, s, sp, 0.001), 0.001);
    past = std::max(past, s.p.x - 10);
    if (i == 3000 || i == 5000 || i == 15000) std::printf("x at %.0f s %.4f m   ", i * 0.001, s.p.x);
  }
  std::printf("overshoot %.4f m\n", past);
}

// average time for one control() call on a drone hovering in place
static double control_ns() {
  QuadParams P;
  Controller c;
  QuadState s;
  s.p = {0, 0, 10};
  Setpoint sp;
  sp.pos = {1, 1, 11};
  const int calls = 2000000;
  double sink = 0;
  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < calls; i++) sink += control(c, P, s, sp, 0.001)[i & 3];
  auto t1 = std::chrono::steady_clock::now();
  // printed so the compiler can't skip the loop
  if (sink == -1) std::printf("%f\n", sink);
  return std::chrono::duration<double, std::nano>(t1 - t0).count() / calls;
}

int main() {
  // step responses with the default gains, same scoring as the tests
  std::printf("step responses (1 m in x, y, z, 1 rad in yaw, 10 s each)\n");
  const char* names[] = {"x", "y", "z", "yaw"};
  for (int a = 0; a < 4; a++) {
    StepInfo r = step_response(Gains{}, a, 1.0, 10.0);
    std::printf("  %-3s overshoot %4.1f%%  settle 2%% %.2f s  final error %.4f  altitude dip %.4f m\n", names[a],
                r.overshoot * 100, r.settle, r.final_err, r.max_dip);
  }
  // the route at a slow and a fast speed
  double length = 0;
  for (size_t k = 0; k + 1 < kRoute.size(); k++) length += norm(kRoute[k + 1] - kRoute[k]);
  std::printf("route (%zu waypoints, %.1f m), the drone chases a reference point moving along it\n", kRoute.size(), length);
  // sharp corners at constant speed, then stopping at each waypoint with 3 m/s^2 ramps
  for (double accel : {0.0, 3.0}) {
    std::printf(accel == 0 ? "  constant speed, instant 90 degree corners\n" : "  3 m/s^2 ramps, stops at each waypoint\n");
    for (double v : {2.0, 5.0}) {
      RouteInfo r = fly_route(v, accel);
      std::printf("    %.0f m/s  %.1f s  rms to reference %.3f m  max %.3f m  rms cross track %.3f m\n", v, r.seconds,
                  r.rms_ref, r.max_ref, r.rms_cross);
    }
  }
  std::printf("recovering from a tipped start\n");
  for (double angle : {0.5236, 2.0944, 3.1416}) recovery(angle);
  std::printf("10 m step in x\n");
  big_step();
  std::printf("control() call: %.0f ns\n", control_ns());
}
