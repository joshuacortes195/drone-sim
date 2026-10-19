#include "doctest.h"
#include "quad.h"

#include <cmath>

// pi without relying on kPi, which msvc doesn't have by default
constexpr double kPi = 3.14159265358979323846;

// rotor order follows rotors' iris: 0 front right, 1 back left, 2 front left, 3 back right
static const QuadParams P;

// rotor speed where all four together hold up the drone's weight
static double hover_speed() { return std::sqrt(P.mass * 9.81 / (4 * P.k_f)); }

// a drone sitting still at 10 m with every rotor already spinning at speed w
static QuadState at_rest(double w = 0) {
  QuadState s;
  s.p = {0, 0, 10};
  s.rotor = {w, w, w, w};
  return s;
}

// runs the sim forward holding the same rotor command the whole time
static QuadState fly(QuadState s, Rotors cmd, double seconds) {
  int steps = (int)std::lround(seconds / 0.001);
  for (int i = 0; i < steps; i++) s = step(P, s, cmd, 0.001);
  return s;
}

TEST_CASE("airframe numbers match the plan: hover near 660 rad/s, thrust to weight 1.61") {
  CHECK(hover_speed() == doctest::Approx(660.36).epsilon(1e-4));
  double max = P.max_rotor_speed;
  CHECK(wrench(P, at_rest(max)).force.z / (P.mass * 9.81) == doctest::Approx(1.6104).epsilon(1e-4));
}

TEST_CASE("equal rotors at hover speed hold position with no rotation for 10 s") {
  double w = hover_speed();
  QuadState s = fly(at_rest(w), {w, w, w, w}, 10.0);
  CHECK(std::abs(s.p.x) < 1e-9);
  CHECK(std::abs(s.p.y) < 1e-9);
  CHECK(std::abs(s.p.z - 10) < 1e-6);
  CHECK(norm(s.w) < 1e-12);
  CHECK(s.q.w == doctest::Approx(1.0));
}

// rotor speed that makes a given thrust
static double speed_for(double thrust) { return std::sqrt(thrust / P.k_f); }

TEST_CASE("front left alone 1 N faster gives roll +0.22, pitch -0.13, yaw +0.016") {
  double th = P.mass * 9.81 / 4;
  QuadState s = at_rest(hover_speed());
  s.rotor[2] = speed_for(th + 1.0);
  Wrench f = wrench(P, s);
  CHECK(f.torque.x == doctest::Approx(0.22));
  CHECK(f.torque.y == doctest::Approx(-0.13));
  CHECK(f.torque.z == doctest::Approx(0.016));
  CHECK(f.force.z == doctest::Approx(P.mass * 9.81 + 1.0));
}

TEST_CASE("clockwise pair up and counterclockwise pair down yaws left, plus the uneven arm roll") {
  // 0.5 N more on front left and back right, 0.5 N less on front right and back left
  double th = P.mass * 9.81 / 4, up = speed_for(th + 0.5), down = speed_for(th - 0.5);
  QuadState s = at_rest();
  s.rotor = {down, down, up, up};
  Wrench f = wrench(P, s);
  CHECK(f.torque.z == doctest::Approx(0.032));  // 4 * 0.016 * 0.5
  CHECK(f.torque.x == doctest::Approx(0.02));   // 0.04 * 0.5 from 0.22 vs 0.20 arms
  CHECK(std::abs(f.torque.y) < 1e-12);
  CHECK(f.force.z == doctest::Approx(P.mass * 9.81));
  // holding it for 0.1 s turns the nose left at about 0.032 / Izz * 0.1 = 0.0328 rad/s
  QuadState later = fly(s, s.rotor, 0.1);
  CHECK(std::abs(later.w.z - 0.0328) < 0.05 * 0.0328);  // approx would allow +-0.05 absolute here
}

TEST_CASE("motors off from rest it falls as z0 - g t^2 / 2") {
  QuadState s = fly(at_rest(), {0, 0, 0, 0}, 2.0);
  CHECK(s.p.z == doctest::Approx(10 - 0.5 * 9.81 * 4).epsilon(1e-12));
  CHECK(s.v.z == doctest::Approx(-9.81 * 2).epsilon(1e-12));
  CHECK(std::abs(s.p.x) + std::abs(s.p.y) < 1e-12);
}

TEST_CASE("pitched 10 degrees nose down at hover thrust it speeds up toward +x at g sin 10") {
  double w = hover_speed();
  QuadState s = at_rest(w);
  s.q = axis_angle({0, 1, 0}, 10 * kPi / 180);
  CHECK(rotate(s.q, {1, 0, 0}).z < 0);  // positive pitch really is nose down
  QuadState later = fly(s, s.rotor, 0.01);
  CHECK(later.v.x / 0.01 == doctest::Approx(1.70349).epsilon(1e-3));
}

TEST_CASE("yawed 90 degrees, a body roll rate turns the drone about world y") {
  QuadState s = at_rest();
  s.q = axis_angle({0, 0, 1}, kPi / 2);  // nose points along world +y
  s.w = {1, 0, 0};
  QuadState later = fly(s, {0, 0, 0, 0}, 0.1);
  // the top of the drone tips toward world +x by 0.1 rad
  Vec3 up = rotate(later.q, {0, 0, 1});
  CHECK(up.x == doctest::Approx(std::sin(0.1)));
  CHECK(std::abs(up.y) < 1e-9);
  CHECK(up.z == doctest::Approx(std::cos(0.1)));
}

// spin that the world sees, R I w, which no torque means can't change
static Vec3 world_momentum(const QuadState& s) { return rotate(s.q, mul(P.inertia, s.w)); }
static double spin_energy(const QuadState& s) { return 0.5 * dot(s.w, mul(P.inertia, s.w)); }

TEST_CASE("torque free tumble off the main axes keeps world momentum and energy for 10 s") {
  QuadState s = at_rest();
  s.w = {1, 2, 3};
  Vec3 L0 = world_momentum(s);
  double E0 = spin_energy(s);
  QuadState later = fly(s, {0, 0, 0, 0}, 10.0);
  CHECK(norm(world_momentum(later) - L0) / norm(L0) < 1e-9);
  CHECK(std::abs(spin_energy(later) - E0) / E0 < 1e-9);
  // the wobble term is doing something: the body rate has really changed
  CHECK(norm(later.w - s.w) > 0.1);
}

TEST_CASE("quaternion stays unit length over a long fast tumble") {
  // fast enough that rk4 alone would drift off unit length, so this needs the renormalize
  QuadState s = at_rest();
  s.w = {20, 40, 60};
  for (int i = 0; i < 60000; i++) {
    s = step(P, s, {0, 0, 0, 0}, 0.001);
    double n = std::sqrt(s.q.w * s.q.w + s.q.x * s.q.x + s.q.y * s.q.y + s.q.z * s.q.z);
    REQUIRE(std::abs(n - 1) < 1e-12);
  }
}

TEST_CASE("motors chase their command with a 12.5 ms lag speeding up and 25 ms slowing down") {
  // 600 rad/s asked from a standstill: 600 (1 - e^(-13/12.5)) after 13 ms
  QuadState up = fly(at_rest(), {600, 600, 600, 600}, 0.013);
  CHECK(up.rotor[0] == doctest::Approx(387.927).epsilon(1e-5));
  // cut from 600 to 0: 600 e^(-13/25) after 13 ms
  QuadState down = fly(at_rest(600), {0, 0, 0, 0}, 0.013);
  CHECK(down.rotor[3] == doctest::Approx(356.712).epsilon(1e-5));
}

TEST_CASE("rotor commands get capped at 838 rad/s and can't go below zero") {
  QuadState s = fly(at_rest(), {2000, 2000, 2000, 2000}, 0.5);
  CHECK(s.rotor[1] == doctest::Approx(838).epsilon(1e-6));
  QuadState r = fly(at_rest(), {-100, -100, -100, -100}, 0.5);
  CHECK(r.rotor[0] == 0.0);
}

TEST_CASE("flying level at 5 m/s, rotor drag slows it and tips the nose up") {
  QuadState s = at_rest(hover_speed());
  s.v = {5, 0, 0};
  Wrench f = wrench(P, s);
  CHECK(f.force.x == doctest::Approx(-1.06506).epsilon(1e-5));    // 4 * 8.06428e-5 * 660.36 * 5
  CHECK(f.force.z == doctest::Approx(P.mass * 9.81));              // drag has no up part here
  CHECK(f.torque.y == doctest::Approx(-0.0244964).epsilon(1e-5));  // drag 2.3 cm above the center
  CHECK(f.torque.x == doctest::Approx(-0.0132072).epsilon(1e-5));  // rotors' rolling moment, all four add up
  CHECK(std::abs(f.torque.z) < 1e-12);
}

TEST_CASE("yawing on the spot, rotor drag on each moving rotor pushes back against the spin") {
  QuadState s = at_rest(hover_speed());
  s.w = {0, 0, 1};
  Wrench f = wrench(P, s);
  CHECK(f.torque.z == doctest::Approx(-0.0130151).epsilon(1e-5));  // -k w sum(x^2 + y^2)
  CHECK(std::abs(f.torque.x) + std::abs(f.torque.y) < 1e-12);
  CHECK(std::abs(f.force.x) + std::abs(f.force.y) < 1e-12);
}
