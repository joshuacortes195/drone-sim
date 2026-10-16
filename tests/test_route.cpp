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
