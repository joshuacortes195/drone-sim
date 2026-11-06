#include "route.h"

#include <algorithm>
#include <cmath>

// how far along a leg of length L the reference is after t seconds, how fast it's going and speeding up.
// accel 0 means it jumps straight to cruise speed and turns corners instantly, otherwise it speeds up
// and slows down at accel so it stops at each waypoint (a triangle if the leg is too short to reach cruise)
void along(double L, double cruise, double accel, double t, double& dist, double& speed, double& acc) {
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
double leg_time(double L, double cruise, double accel) {
  if (accel == 0) return L / cruise;
  double top = std::min(cruise, std::sqrt(accel * L));
  return L / top + top / accel;
}

// fastest a leg may go up or down: 3/4 of the controller's vertical speed cap, so the feedback still has room
// to catch up. a steep leg at full cruise asked for 4.2 m/s down and fell 7 m behind
static const double kMaxClimb = 0.75 * Gains{}.max_speed_z;

// cruise speed for one leg, slowed down when its up or down part would go past kMaxClimb
static double leg_cruise(Vec3 d, double cruise) {
  double L = norm(d);
  return std::abs(d.z) * cruise > kMaxClimb * L ? kMaxClimb * L / std::abs(d.z) : cruise;
}

// total seconds to fly a whole path, stopping at every waypoint
double path_time(const std::vector<Vec3>& path, double cruise, double accel, Vec3 v0) {
  // the braking leg first, if it starts at speed, then the rest from its end
  double T = 0;
  size_t first = 1;
  if (norm(v0) > 0 && path.size() >= 2) T = norm(v0) / kBrake, first = 2;
  for (size_t i = first; i < path.size(); i++) {
    Vec3 d = path[i] - path[i - 1];
    if (norm(d) > 0) T += leg_time(norm(d), leg_cruise(d, cruise), accel);  // a repeated point takes no time
  }
  return T;
}

Setpoint follow(const std::vector<Vec3>& path, double t, double cruise, double accel, double yaw, Vec3 v0) {
  Setpoint sp;
  sp.yaw = yaw;
  if (path.empty()) return sp;
  // starting at speed: the first leg slows down at kBrake along the way it's going, to rest on the second point
  double v_start = norm(v0);
  if (v_start > 0 && path.size() >= 2) {
    double tb = v_start / kBrake;
    Vec3 u = v0 * (1 / v_start);
    if (t < tb) {
      sp.pos = path[0] + v0 * t - u * (kBrake * t * t / 2);
      sp.vel = v0 - u * (kBrake * t);
      sp.acc = u * -kBrake;
      return sp;
    }
    // after that it's the rest of the path from where the braking ended, just that much later
    return follow(std::vector<Vec3>(path.begin() + 1, path.end()), t - tb, cruise, accel, yaw);
  }
  sp.pos = path.back();
  for (size_t i = 1; i < path.size(); i++) {
    Vec3 d = path[i] - path[i - 1];
    double L = norm(d);
    if (L == 0) continue;  // a repeated point takes no time
    // nose along this leg, a straight up or down leg keeps the heading it already had
    if (std::hypot(d.x, d.y) > 1e-6) sp.yaw = std::atan2(d.y, d.x);
    double v = leg_cruise(d, cruise);
    double T = leg_time(L, v, accel);
    if (t < T) {
      // somewhere on this leg: slide along its direction
      double dist, speed, acc;
      along(L, v, accel, t, dist, speed, acc);
      Vec3 u = d * (1 / L);
      sp.pos = path[i - 1] + u * dist;
      sp.vel = u * speed;
      sp.acc = u * acc;
      return sp;
    }
    t -= T;
  }
  // past the end: hold the last point, still facing along the last leg that moved sideways
  return sp;
}

std::vector<Vec3> lawnmower(double x0, double y0, double x1, double y1, double z, double spacing) {
  // enough gaps that none is wider than spacing, then spread the rows evenly so both edges get a row
  int gaps = int(std::ceil((y1 - y0) / spacing - 1e-9));
  std::vector<Vec3> pts;
  for (int r = 0; r <= gaps; r++) {
    double y = gaps == 0 ? y0 : y0 + (y1 - y0) * r / gaps;
    // even rows go west to east, odd rows come back
    double a = r % 2 == 0 ? x0 : x1, b = r % 2 == 0 ? x1 : x0;
    pts.push_back({a, y, z});
    pts.push_back({b, y, z});
  }
  return pts;
}
