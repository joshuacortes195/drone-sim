#include "sensors.h"

#include <cmath>

// a bell curve random number with spread 1. built by hand from raw mt19937 draws (box muller)
// because the std distributions give different numbers on msvc and in the browser build
double gauss(std::mt19937& rng) {
  double u1 = (rng() + 1.0) / 4294967296.0;  // (0, 1], never 0 so the log is safe
  double u2 = rng() / 4294967296.0;
  return std::sqrt(-2 * std::log(u1)) * std::cos(2 * 3.14159265358979323846 * u2);
}

// three independent bell curve numbers, each scaled by its own spread
static Vec3 gauss3(std::mt19937& rng, Vec3 sigma) {
  double x = gauss(rng), y = gauss(rng), z = gauss(rng);
  return {x * sigma.x, y * sigma.y, z * sigma.z};
}

Noise quiet() {
  Noise n;
  n.accel_density = {0, 0, 0};
  n.gyro_density = n.accel_bias0 = n.gyro_bias0 = n.accel_walk = n.gyro_walk = 0;
  n.gps_xy = n.gps_z = n.mag = n.lidar = 0;
  return n;
}

Sensors make_sensors(const Noise& n, unsigned seed) {
  Sensors sn;
  sn.n = n;
  sn.rng.seed(seed);
  // every chip powers up a little off, by a different amount each time
  double a = n.accel_bias0, g = n.gyro_bias0;
  sn.accel_bias = gauss3(sn.rng, {a, a, a});
  sn.gyro_bias = gauss3(sn.rng, {g, g, g});
  return sn;
}

Imu read_imu(Sensors& sn, const QuadParams& P, const QuadState& s, double dt, Vec3 wind) {
  const Noise& n = sn.n;
  // noise per sample grows as samples get closer together: density / sqrt(dt)
  double per = 1 / std::sqrt(dt);
  double g = n.gyro_density * per;
  // the accelerometer feels thrust and rotor drag (wind included) but never gravity
  Vec3 accel = wrench(P, s, wind).force * (1 / P.mass) + sn.accel_bias + gauss3(sn.rng, n.accel_density * per);
  Vec3 gyro = s.w + sn.gyro_bias + gauss3(sn.rng, {g, g, g});
  // the biases wander a little every sample, a random walk: walk * sqrt(dt) per step
  double aw = n.accel_walk * std::sqrt(dt), gw = n.gyro_walk * std::sqrt(dt);
  sn.accel_bias = sn.accel_bias + gauss3(sn.rng, {aw, aw, aw});
  sn.gyro_bias = sn.gyro_bias + gauss3(sn.rng, {gw, gw, gw});
  return {accel, gyro};
}

Vec3 read_gps(Sensors& sn, const QuadState& s) {
  // fresh noise each fix, worse up and down than across
  return s.p + gauss3(sn.rng, {sn.n.gps_xy, sn.n.gps_xy, sn.n.gps_z});
}

double read_mag(Sensors& sn, const QuadState& s) {
  // the true heading plus noise, kept in -pi..pi
  double h = heading(s.q) + sn.n.mag * gauss(sn.rng);
  return std::atan2(std::sin(h), std::cos(h));
}

Noise rtk() {
  Noise n;
  n.gps_xy = 0.0093;  // 0.011 m cep50 / 1.1774
  n.gps_z = 0.0163;   // 0.011 m median / 0.6745
  return n;
}
