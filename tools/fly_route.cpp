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

// how far along a leg of length L the reference is after t seconds, how fast it's going and speeding up.
// accel 0 means it jumps straight to cruise speed and turns corners instantly, otherwise it speeds up
// and slows down at accel so it stops at each waypoint (a triangle if the leg is too short to reach cruise)
static void along(double L, double cruise, double accel, double t, double& dist, double& speed, double& acc) {
  acc = 0;
  if (accel == 0) {
    dist = cruise * t;
    speed = cruise;
    return;
  }
  double top = std::min(cruise, std::sqrt(accel * L));
  double t_ramp = top / accel, t_total = L / top + t_ramp;
  if (t < t_ramp) {
    dist = accel * t * t / 2;
    speed = accel * t;
    acc = accel;
  } else if (t < t_total - t_ramp) {
    dist = top * t_ramp / 2 + top * (t - t_ramp);
    speed = top;
  } else {
    double left = std::max(t_total - t, 0.0);
    dist = L - accel * left * left / 2;
    speed = accel * left;
    acc = -accel;
  }
}

// seconds the reference needs for a leg of length L
static double leg_time(double L, double cruise, double accel) {
  if (accel == 0) return L / cruise;
  double top = std::min(cruise, std::sqrt(accel * L));
  return L / top + top / accel;
}

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
  std::printf("control() call: %.0f ns\n", control_ns());
}
