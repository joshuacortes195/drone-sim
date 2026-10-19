#include "doctest.h"
#include "controller.h"
#include "estimator.h"

#include <algorithm>
#include <cmath>

static const QuadParams P;

// a whole drone: the real one, its controller, its sensors and what it thinks its state is
struct Flight {
  QuadState s;
  Controller c;
  Sensors sn;
  Estimator e;
  bool truth = false;  // fly on the real state instead of the estimate
  double max_pos_err = 0, max_att_err = 0;
  double sq_pos_err = 0, sq_gps_err = 0;  // summed at each gps fix for rms
  int fixes = 0;
};

// starts sitting still in a hover at 5 m, with the estimator told where it is
static Flight start(const Noise& n, unsigned seed) {
  Flight f;
  f.s.p = {0, 0, 5};
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  f.s.rotor = {w, w, w, w};
  f.sn = make_sensors(n, seed);
  f.e = make_estimator(f.s, n);
  // gyro calibration on the pad, like a real flight controller at power up: averages 1 s of gyro while still
  Vec3 sum;
  for (int i = 0; i < 1000; i++) sum = sum + read_imu(f.sn, P, f.s, 0.001).gyro;
  f.e.gyro_bias = sum * 0.001;
  return f;
}

// angle between the real and estimated attitude
static double att_err(Quat a, Quat b) {
  Quat d = conj(a) * b;
  return 2 * std::atan2(std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z), std::abs(d.w));
}

// one second at a time of: read sensors, update the estimate, control, move. gps and mag at 10 Hz
static void fly(Flight& f, Setpoint sp, double seconds) {
  const double dt = 0.001;
  int steps = (int)std::lround(seconds / dt);
  for (int i = 0; i < steps; i++) {
    predict(f.e, read_imu(f.sn, P, f.s, dt), dt);
    if (i % 100 == 0) {
      Vec3 fix = read_gps(f.sn, f.s);
      gps(f.e, fix);
      mag(f.e, read_mag(f.sn, f.s));
      f.sq_pos_err += dot(f.e.p - f.s.p, f.e.p - f.s.p);
      f.sq_gps_err += dot(fix - f.s.p, fix - f.s.p);
      f.fixes++;
    }
    f.max_pos_err = std::max(f.max_pos_err, norm(f.e.p - f.s.p));
    f.max_att_err = std::max(f.max_att_err, att_err(f.e.q, f.s.q));
    Rotors cmd = control(f.c, P, f.truth ? f.s : estimate(f.e), sp, dt);
    f.s = step(P, f.s, cmd, dt);
  }
}

// a box of moves: 5 m east and 3 m up while turning 1 rad, then back
static void maneuver(Flight& f) {
  Setpoint a;
  a.pos = {5, 0, 8};
  a.yaw = 1;
  fly(f, a, 8);
  Setpoint b;
  b.pos = {0, 0, 5};
  fly(f, b, 8);
}

TEST_CASE("with perfect sensors the estimate stays on the truth through a maneuver") {
  Flight f = start(quiet(), 1);
  maneuver(f);
  CHECK(f.max_pos_err < 0.02);
  CHECK(f.max_att_err < 0.005);  // the weak tilt pull while speeding up is all that's left
  // and the drone really went there, flying on the estimate
  CHECK(norm(f.s.p - Vec3{0, 0, 5}) < 0.1);
}

TEST_CASE("with real sensor noise the kalman filter beats raw gps by more than 3x") {
  for (unsigned seed = 1; seed <= 4; seed++) {
    Flight f = start(Noise{}, seed);
    maneuver(f);
    Setpoint hold;
    hold.pos = {0, 0, 5};
    fly(f, hold, 44);
    double kf = std::sqrt(f.sq_pos_err / f.fixes), raw = std::sqrt(f.sq_gps_err / f.fixes);
    CHECK(kf < raw / 3);
    CHECK(f.max_att_err < 0.05);  // about the sideways accel bias over g, which hover can't tell from tilt
  }
}

TEST_CASE("the kalman filter learns a constant up and down accelerometer bias") {
  Noise n;
  n.accel_bias0 = n.accel_walk = 0;
  Flight f = start(n, 5);
  f.sn.accel_bias = {0, 0, 0.3};
  Setpoint hold;
  hold.pos = {0, 0, 5};
  fly(f, hold, 60);
  CHECK(std::abs(f.e.accel_bias.z - 0.3) < 0.03);
}

TEST_CASE("the magnetometer stops yaw drifting with a 1 deg/s gyro bias and the bias is learned") {
  Noise n;
  n.gyro_bias0 = 0;
  Flight f = start(n, 6);
  // the bias turns up after calibration, so the estimator has to find it itself
  f.sn.gyro_bias = {0, 0, 0.01745};
  Setpoint hold;
  hold.pos = {0, 0, 5};
  hold.yaw = 0.5;
  fly(f, hold, 90);
  double err = heading(f.e.q) - heading(f.s.q);
  CHECK(std::abs(std::atan2(std::sin(err), std::cos(err))) < 0.02);  // gyro alone would be 1.6 rad off by now
  CHECK(std::abs(f.e.gyro_bias.z - 0.01745) < 0.002);
}

TEST_CASE("in a still hover the accelerometer pulls a wrong tilt estimate back toward level") {
  Flight f = start(quiet(), 1);
  f.truth = true;  // flies on the real state so only the estimate is wrong
  f.e.q = axis_angle({1, 0, 0}, 0.05);
  Setpoint hold;
  hold.pos = {0, 0, 5};
  fly(f, hold, 100);
  CHECK(att_err(f.e.q, f.s.q) < 0.025);
}

TEST_CASE("a big up and down accel bias doesn't switch the tilt pull off") {
  // 0.4 m/s^2 puts the raw reading 4% off 1 g, outside the 3% trust window until the bias is learned
  Flight f = start(quiet(), 1);
  f.truth = true;
  f.sn.accel_bias = {0, 0, 0.4};
  f.e.q = axis_angle({1, 0, 0}, 0.05);
  Setpoint hold;
  hold.pos = {0, 0, 5};
  fly(f, hold, 100);
  CHECK(att_err(f.e.q, f.s.q) < 0.025);
}

TEST_CASE("the tilt pull learns a roll gyro bias that shows up after calibration") {
  Flight f = start(quiet(), 1);
  f.truth = true;
  f.sn.gyro_bias = {0.005, 0, 0};
  Setpoint hold;
  hold.pos = {0, 0, 5};
  fly(f, hold, 300);
  // without the integral it never catches up, 0.71 rad off after 300 s when I tried it
  CHECK(std::abs(f.e.gyro_bias.x - 0.005) < 0.001);
  CHECK(att_err(f.e.q, f.s.q) < 0.01);
}
