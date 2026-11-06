#include "doctest.h"
#include "route.h"

#include <algorithm>
#include <cmath>

// an L shaped path: 10 m east, then 6 m north and 3 m up
static const std::vector<Vec3> L_PATH = {{0, 0, 5}, {10, 0, 5}, {10, 6, 8}};

TEST_CASE("follow: starts at rest on the first point and ends at rest on the last") {
  Setpoint a = follow(L_PATH, 0, 5, 3);
  CHECK(norm(a.pos - L_PATH[0]) < 1e-12);
  CHECK(norm(a.vel) < 1e-12);
  double T = path_time(L_PATH, 5, 3);
  Setpoint b = follow(L_PATH, T + 10, 5, 3);
  CHECK(norm(b.pos - L_PATH[2]) < 1e-12);
  CHECK(norm(b.vel) < 1e-12);
  CHECK(norm(b.acc) < 1e-12);
}

TEST_CASE("follow: leg times worked by hand, stops on the corner, faces along each leg") {
  // leg 1, 10 m at 5 m/s cruise and 3 m/s^2: 5/3 s up, 5/3 s down, (10 - 25/3) / 5 = 1/3 s cruising = 11/3 s
  // leg 2, sqrt(36 + 9) = 6.708 m climbing 3 m: capped so the climb is 1.5 m/s, cruise 1.5 * 6.708 / 3 = 3.354 m/s,
  // 6.708 / 3.354 + 3.354 / 3 = 3.118 s
  CHECK(path_time(L_PATH, 5, 3) == doctest::Approx(11.0 / 3 + 3.118034).epsilon(1e-6));
  Setpoint corner = follow(L_PATH, 11.0 / 3, 5, 3);
  CHECK(norm(corner.pos - L_PATH[1]) < 1e-9);
  CHECK(norm(corner.vel) < 1e-9);
  // halfway down leg 1: heading east at full cruise, 5 m along
  Setpoint mid = follow(L_PATH, 11.0 / 6, 5, 3);
  CHECK(mid.pos.x == doctest::Approx(5));
  CHECK(mid.vel.x == doctest::Approx(5));
  CHECK(mid.yaw == doctest::Approx(0));
  // early in leg 2: speeding up along (0, 6, 3) / 6.708, facing north
  Setpoint up = follow(L_PATH, 11.0 / 3 + 0.5, 5, 3);
  CHECK(up.acc.y == doctest::Approx(3 * 6 / std::sqrt(45.0)));
  CHECK(up.acc.z == doctest::Approx(3 * 3 / std::sqrt(45.0)));
  CHECK(up.yaw == doctest::Approx(1.5707963267948966));
}

TEST_CASE("follow: a straight up leg keeps the heading of the leg before it") {
  std::vector<Vec3> path = {{0, 0, 2}, {0, 5, 2}, {0, 5, 6}};
  CHECK(follow(path, path_time(path, 5, 3) - 0.5, 5, 3).yaw == doctest::Approx(1.5707963267948966));
}

TEST_CASE("follow: a steep leg slows down so it never asks for more than 1.5 m/s up or down") {
  // 11.2 m sideways and 17 m down, the leg that fell 7 m behind in the planning flights
  std::vector<Vec3> path = {{0, 0, 18}, {11.2, 0, 1}};
  double worst = 0;
  for (double t = 0; t < path_time(path, 5, 3); t += 0.01) worst = std::max(worst, std::abs(follow(path, t, 5, 3).vel.z));
  CHECK(worst == doctest::Approx(1.5));
  // a flat leg still gets the full 5 m/s
  std::vector<Vec3> flat = {{0, 0, 5}, {30, 0, 5}};
  CHECK(follow(flat, path_time(flat, 5, 3) / 2, 5, 3).vel.x == doctest::Approx(5));
  // and path_time agrees with how long the setpoint actually takes to get to the end
  double T = path_time(path, 5, 3);
  CHECK(norm(follow(path, T - 0.001, 5, 3).pos - path[1]) < 0.01);
  CHECK(norm(follow(path, T * 0.9, 5, 3).pos - path[1]) > 0.1);
}

TEST_CASE("follow: a repeated point is skipped instead of breaking the timing") {
  std::vector<Vec3> path = {{0, 0, 2}, {0, 0, 2}, {5, 0, 2}};
  std::vector<Vec3> clean = {{0, 0, 2}, {5, 0, 2}};
  CHECK(path_time(path, 5, 3) == doctest::Approx(path_time(clean, 5, 3)));
  CHECK(follow(path, 0.5, 5, 3).pos.x == doctest::Approx(follow(clean, 0.5, 5, 3).pos.x));
}

TEST_CASE("follow: keeps the heading it started with until a leg moves sideways") {
  // straight up first, then east: faces the starting 2 rad on the way up, then turns east
  std::vector<Vec3> path = {{0, 0, 2}, {0, 0, 6}, {5, 0, 6}};
  CHECK(follow(path, 0.5, 5, 3, 2.0).yaw == doctest::Approx(2.0));
  CHECK(follow(path, path_time(path, 5, 3) - 0.5, 5, 3, 2.0).yaw == doctest::Approx(0));
  // a path that's just one point holds there, facing the same way
  Setpoint one = follow({{1, 2, 3}}, 1, 5, 3, -1.0);
  CHECK(norm(one.pos - Vec3{1, 2, 3}) < 1e-12);
  CHECK(one.yaw == doctest::Approx(-1.0));
}

TEST_CASE("lawnmower: rows across the area, back and forth, worked by hand") {
  // 10 m wide, 12 m deep, rows 6 m apart: three rows, each flown the other way from the last
  std::vector<Vec3> want = {{0, 0, 8}, {10, 0, 8}, {10, 6, 8}, {0, 6, 8}, {0, 12, 8}, {10, 12, 8}};
  std::vector<Vec3> got = lawnmower(0, 0, 10, 12, 8, 6);
  REQUIRE(got.size() == want.size());
  for (size_t i = 0; i < want.size(); i++) CHECK(norm(got[i] - want[i]) < 1e-12);
}

TEST_CASE("lawnmower: spacing that doesn't fit evenly still reaches both edges, no gap wider than asked") {
  // 10 m deep at 4 m: three gaps of 3.33 m instead of leaving the last 2 m unsearched
  std::vector<Vec3> got = lawnmower(-5, 0, 5, 10, 3, 4);
  REQUIRE(got.size() == 8);
  CHECK(got.front().y == doctest::Approx(0));
  CHECK(got.back().y == doctest::Approx(10));
  for (size_t i = 0; i < got.size(); i += 2) {
    // each row runs straight across at one y, the full width, and the next one turns back
    CHECK(got[i].y == got[i + 1].y);
    CHECK(std::abs(got[i].x - got[i + 1].x) == doctest::Approx(10));
    if (i + 2 < got.size()) {
      CHECK(got[i + 2].x == got[i + 1].x);
      CHECK(got[i + 2].y - got[i].y == doctest::Approx(10.0 / 3));
    }
  }
  for (const Vec3& p : got) CHECK(p.z == 3);
}

TEST_CASE("lawnmower: an area with no depth is one row") {
  std::vector<Vec3> got = lawnmower(0, 2, 6, 2, 5, 4);
  REQUIRE(got.size() == 2);
  CHECK(norm(got[0] - Vec3{0, 2, 5}) < 1e-12);
  CHECK(norm(got[1] - Vec3{6, 2, 5}) < 1e-12);
}

TEST_CASE("lawnmower: rounding in the division doesn't add a row") {
  // 2.1 / 0.3 comes out a hair over 7 in floating point, it should still be 7 gaps (8 rows), not 8
  std::vector<Vec3> got = lawnmower(0, 0, 1, 2.1, 2, 0.3);
  CHECK(got.size() == 16);
}

// go home ends by coming straight down onto the pad. flown with the real controller and physics from a 2 m hover:
// it never sinks below where the skids touch, gets there gently and doesn't drift or tip on the way down
TEST_CASE("landing: from a 2 m hover it comes straight down to skid height, slow, level and on the spot") {
  QuadParams P;
  Controller c;
  QuadState s;
  s.p = {3, -4, 2};
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  s.rotor = {w, w, w, w};
  std::vector<Vec3> down = {s.p, {3, -4, kSkid}};
  double T = path_time(down, kLandSpeed, kLandSpeed);
  double lowest = 10, fastest = 0, drift = 0, tilt = 0;
  for (int i = 1; i <= int((T + 2) / 0.001); i++) {
    s = step(P, s, control(c, P, s, follow(down, i * 0.001, kLandSpeed, kLandSpeed), 0.001), 0.001);
    lowest = std::min(lowest, s.p.z);
    fastest = std::max(fastest, std::abs(s.v.z));
    drift = std::max(drift, std::hypot(s.p.x - 3, s.p.y + 4));
    tilt = std::max(tilt, std::acos(std::clamp(rotate(s.q, {0, 0, 1}).z, -1.0, 1.0)));
  }
  CHECK(lowest > kSkid - 0.01);
  CHECK(std::abs(s.p.z - kSkid) < 0.01);
  CHECK(fastest < kLandSpeed * 1.1);
  CHECK(drift < 0.01);
  CHECK(tilt < 0.01);
}
