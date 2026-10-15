#include "doctest.h"
#include "controller.h"
#include "step_response.h"

#include <algorithm>
#include <array>
#include <cmath>

static const QuadParams P;

// what the rotors actually push with, read back through the phase 1 physics with the drone at rest
static Wrench push_from(Rotors r) {
  QuadState s;
  s.rotor = r;
  return wrench(P, s);
}

// true when some rotor sits right at 0 or full speed, meaning the mixer used everything it had
static bool some_rotor_maxed(Rotors r) {
  for (double w : r)
    if (w < 1e-6 || w > P.max_rotor_speed - 1e-6) return true;
  return false;
}

static bool all_in_range(Rotors r) {
  for (double w : r)
    if (w < 0 || w > P.max_rotor_speed + 1e-9) return false;
  return true;
}

TEST_CASE("mixer round trip: rotor speeds give back the thrust and torque that were asked for") {
  Wrench f = push_from(mix(P, 15.0, {0.1, -0.05, 0.02}));
  CHECK(f.force.z == doctest::Approx(15.0).epsilon(1e-9));
  CHECK(f.torque.x == doctest::Approx(0.1).epsilon(1e-9));
  CHECK(f.torque.y == doctest::Approx(-0.05).epsilon(1e-9));
  CHECK(f.torque.z == doctest::Approx(0.02).epsilon(1e-9));
}

TEST_CASE("mixer: a yaw request too big to do gives way, thrust and roll stay exact") {
  Rotors r = mix(P, 15.0, {0.1, 0, 1.0});
  Wrench f = push_from(r);
  CHECK(all_in_range(r));
  CHECK(some_rotor_maxed(r));
  CHECK(f.force.z == doctest::Approx(15.0).epsilon(1e-9));
  CHECK(f.torque.x == doctest::Approx(0.1).epsilon(1e-9));
  CHECK(std::abs(f.torque.y) < 1e-9);
  CHECK(f.torque.z > 0.05);
  CHECK(f.torque.z < 1.0);
}

TEST_CASE("mixer: a roll request too big to do shrinks roll and drops yaw, thrust stays exact") {
  Rotors r = mix(P, 15.0, {5.0, 0, 0.05});
  Wrench f = push_from(r);
  CHECK(all_in_range(r));
  CHECK(some_rotor_maxed(r));
  CHECK(f.force.z == doctest::Approx(15.0).epsilon(1e-9));
  CHECK(f.torque.x > 0.5);
  CHECK(f.torque.x < 5.0);
  CHECK(std::abs(f.torque.y) < 1e-9);
  CHECK(std::abs(f.torque.z) < 1e-9);
}

TEST_CASE("mixer: thrust past what four rotors can make clamps to full speed, never past it") {
  Rotors r = mix(P, 30.0, {0, 0, 0});
  for (double w : r) CHECK(w == doctest::Approx(P.max_rotor_speed));
  r = mix(P, -5.0, {0, 0, 0});
  for (double w : r) CHECK(w == 0.0);
}

// flies the drone under the controller for a while, one 1 ms physics step at a time
static QuadState fly(Controller& c, QuadState s, const Setpoint& sp, double seconds) {
  int steps = (int)std::lround(seconds / 0.001);
  for (int i = 0; i < steps; i++) s = step(P, s, control(c, P, s, sp, 0.001), 0.001);
  return s;
}

// how far the drone is tipped from level, radians (angle between body z and world z)
static double tilt(Quat q) { return std::acos(std::clamp(rotate(q, {0, 0, 1}).z, -1.0, 1.0)); }

// a drone hanging still at 10 m with its rotors already at hover speed
static QuadState hovering() {
  QuadState s;
  s.p = {0, 0, 10};
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  s.rotor = {w, w, w, w};
  return s;
}

TEST_CASE("hover hold: the controller keeps the drone on its set point for 10 s") {
  Controller c;
  Setpoint sp;
  sp.pos = {0, 0, 10};
  QuadState s = fly(c, hovering(), sp, 10.0);
  CHECK(norm(s.p - sp.pos) < 1e-3);
  CHECK(norm(s.v) < 1e-3);
  CHECK(tilt(s.q) < 1e-3);
}

TEST_CASE("hover hold: starting rolled 30 degrees it levels out and comes back to the set point") {
  Controller c;
  Setpoint sp;
  sp.pos = {0, 0, 10};
  QuadState s = hovering();
  s.q = axis_angle({1, 0, 0}, 0.5236);
  // tracks the worst it strays while recovering
  double worst = 0;
  for (int i = 0; i < 6000; i++) {
    s = step(P, s, control(c, P, s, sp, 0.001), 0.001);
    worst = std::max(worst, norm(s.p - sp.pos));
  }
  CHECK(worst < 0.3);
  // the velocity integral charged up in the first lurch takes a few seconds to bleed off, so 5 cm not 1 mm
  CHECK(norm(s.p - sp.pos) < 0.05);
  CHECK(tilt(s.q) < 0.01);
}

// limits agreed for phase 2: under 10% overshoot, inside 2% by the settle time, under 1 cm (or 0.01 rad)
// off at the end, and altitude never moving more than 5 cm while stepping another axis
static void check_step(int axis, double settle_max) {
  StepInfo r = step_response(Gains{}, axis, 1.0, 10.0);
  CHECK(r.overshoot < 0.10);
  CHECK(r.settle < settle_max);
  CHECK(r.final_err < 0.01);
  CHECK(r.max_dip < 0.05);
}

TEST_CASE("1 m step in x: under 10% overshoot, settled in 4 s") { check_step(0, 4.0); }
TEST_CASE("1 m step in y: under 10% overshoot, settled in 4 s") { check_step(1, 4.0); }
TEST_CASE("1 m step in z: under 10% overshoot, settled in 3 s") { check_step(2, 3.0); }
TEST_CASE("1 rad step in yaw: under 10% overshoot, settled in 4 s") { check_step(3, 4.0); }

// flies to a goal 10 m away after someone holds the drone still for hold_ms, reports the worst
// overshoot and tilt, and the x position every 100 ms so two runs can be compared
struct Release {
  double past = 0, worst_tilt = 0;
  std::array<double, 150> x{};
};
static Release let_go(int hold_ms) {
  Controller c;
  Setpoint sp;
  sp.pos = {10, 0, 10};
  QuadState s = hovering();
  // held still, the controller keeps asking and its integrals keep seeing error. hold_ms is a multiple
  // of outer_every (20) so the slow loop ticks at the same moments as in the free run
  for (int i = 0; i < hold_ms; i++) control(c, P, s, sp, 0.001);
  Release r;
  for (int i = 1; i <= 15000; i++) {
    s = step(P, s, control(c, P, s, sp, 0.001), 0.001);
    r.past = std::max(r.past, s.p.x - 10);
    r.worst_tilt = std::max(r.worst_tilt, tilt(s.q));
    if (i % 100 == 0) r.x[i / 100 - 1] = s.p.x;
  }
  return r;
}

TEST_CASE("no windup: held 5 s with the goal 10 m away, it flies the same as if it was never held") {
  Release held = let_go(5000), free = let_go(0);
  CHECK(held.past < 0.5);
  CHECK(held.worst_tilt < 0.698);  // 40 degrees, the 35 degree limit plus room for the attitude loop
  double gap = 0;
  for (int i = 0; i < 150; i++) gap = std::max(gap, std::abs(held.x[i] - free.x[i]));
  CHECK(gap < 0.01);
}

TEST_CASE("feed forward: chasing a point that speeds up at 3 m/s^2 to 5 m/s, it stays within 30 cm") {
  Controller c;
  QuadState s = hovering();
  double worst = 0;
  for (int i = 1; i <= 5000; i++) {
    double t = i * 0.001, ramp = 5.0 / 3.0;
    // the reference speeds up along x, then cruises at 5 m/s
    Setpoint sp;
    sp.pos = {t < ramp ? 1.5 * t * t : 1.5 * ramp * ramp + 5 * (t - ramp), 0, 10};
    sp.vel = {t < ramp ? 3 * t : 5, 0, 0};
    sp.acc = {t < ramp ? 3.0 : 0.0, 0, 0};
    s = step(P, s, control(c, P, s, sp, 0.001), 0.001);
    worst = std::max(worst, norm(s.p - sp.pos));
  }
  CHECK(worst < 0.3);
}

// starts hovering but tipped by `angle` about x, flies 10 s, reports the worst height lost and the end state
static QuadState recover_from(double angle, double& lowest) {
  Controller c;
  Setpoint sp;
  sp.pos = {0, 0, 10};
  QuadState s = hovering();
  s.q = axis_angle({1, 0, 0}, angle);
  lowest = s.p.z;
  for (int i = 0; i < 10000; i++) {
    s = step(P, s, control(c, P, s, sp, 0.001), 0.001);
    lowest = std::min(lowest, s.p.z);
  }
  return s;
}

TEST_CASE("flipped 120 degrees or fully upside down, it rights itself and gets back to the set point") {
  for (double angle : {2.094, 3.1416}) {
    CAPTURE(angle);
    double lowest;
    QuadState s = recover_from(angle, lowest);
    CHECK(tilt(s.q) < 0.01);
    CHECK(norm(s.p - Vec3{0, 0, 10}) < 0.05);
    CHECK(lowest > 0);  // never lost more than the 10 m it started with
  }
}

TEST_CASE("1 m step down in z: under 10% overshoot, settled in 3 s (down is slower motors, not a mirror of up)") {
  StepInfo r = step_response(Gains{}, 2, -1.0, 10.0);
  CHECK(r.overshoot < 0.10);
  CHECK(r.settle < 3.0);
  CHECK(r.final_err < 0.01);
}

TEST_CASE("yaw from -3.0 to 3.0 rad turns the short way through 180 degrees, not back through 0") {
  Controller c;
  Setpoint sp;
  sp.pos = {0, 0, 10};
  sp.yaw = 3.0;
  QuadState s = hovering();
  s.q = axis_angle({0, 0, 1}, -3.0);
  // the short way is 0.28 rad through +-pi, so the heading should never get near 0
  double closest_to_zero = 10;
  for (int i = 0; i < 5000; i++) {
    s = step(P, s, control(c, P, s, sp, 0.001), 0.001);
    closest_to_zero = std::min(closest_to_zero, std::abs(yaw_of(s.q)));
  }
  CHECK(closest_to_zero > 2.9);
  CHECK(std::abs(yaw_of(s.q) - 3.0) < 0.01);
}

TEST_CASE("facing 90 degrees left, a 1 m step in x still flies along world x") {
  Controller c;
  Setpoint sp;
  sp.pos = {1, 0, 10};
  sp.yaw = 1.5708;
  QuadState s = hovering();
  s.q = axis_angle({0, 0, 1}, 1.5708);
  double worst_y = 0;
  for (int i = 0; i < 6000; i++) {
    s = step(P, s, control(c, P, s, sp, 0.001), 0.001);
    worst_y = std::max(worst_y, std::abs(s.p.y));
  }
  CHECK(worst_y < 0.02);
  CHECK(norm(s.p - sp.pos) < 0.01);
}
