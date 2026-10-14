#pragma once

#include "controller.h"

#include <algorithm>
#include <cmath>

// how one step response went
struct StepInfo {
  double overshoot;  // past the target, as a share of the step size
  double settle;     // last time it was outside the 2% band, s
  double final_err;  // distance from the target at the end
  double max_dip;    // worst altitude change while stepping a different axis, m
};

// heading of the drone's nose, radians
inline double yaw_of(Quat q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

// flies a hovering drone at 10 m to a set point moved by `step` along one axis (0 x, 1 y, 2 z, 3 yaw)
// for `seconds` on the 1 ms physics step, and scores how it got there
inline StepInfo step_response(const Gains& g, int axis, double size, double seconds) {
  QuadParams P;
  Controller c;
  c.g = g;
  QuadState s;
  s.p = {0, 0, 10};
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  s.rotor = {w, w, w, w};
  Setpoint sp;
  sp.pos = s.p;
  if (axis == 0) sp.pos.x += size;
  if (axis == 1) sp.pos.y += size;
  if (axis == 2) sp.pos.z += size;
  if (axis == 3) sp.yaw = size;
  // reads the axis being stepped
  auto value = [&](const QuadState& st) {
    if (axis == 0) return st.p.x;
    if (axis == 1) return st.p.y;
    if (axis == 2) return st.p.z - 10;
    return yaw_of(st.q);
  };
  StepInfo r{0, 0, 0, 0};
  int steps = (int)std::lround(seconds / 0.001);
  for (int i = 1; i <= steps; i++) {
    s = step(P, s, control(c, P, s, sp, 0.001), 0.001);
    double v = value(s);
    r.overshoot = std::max(r.overshoot, (v - size) / size);
    if (std::abs(v - size) > 0.02 * std::abs(size)) r.settle = i * 0.001;
    if (axis != 2) r.max_dip = std::max(r.max_dip, std::abs(s.p.z - 10));
  }
  r.final_err = std::abs(value(s) - size);
  return r;
}
