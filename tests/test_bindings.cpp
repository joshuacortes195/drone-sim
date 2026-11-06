#include "doctest.h"

#include <cmath>

// the page's side of the sim (wasm/bindings.cpp), built natively so the landing can be flown the way the page flies it
extern "C" {
void reset(double x, double y, double z, double yaw);
int new_city(unsigned seed);
void advance(int steps);
int fly_to(double x, double y, double z);
int path_done();
void land(double x, double y);
int is_landed();
int is_crashed();
double* state_now();
double sim_tick();
}

// steps the sim up to s seconds, stops early on a crash or once done() says so
template <class F>
static void run(double s, F done) {
  for (int i = 0; i < int(s * 1000) && !is_crashed() && !done(); i++) advance(1);
}

// the page's start: the seed 1 city, hovering 1.5 m over home, then 2 m over the pad and settled
static void hover_over_home() {
  new_city(1);
  reset(0, 0, 1.5, 0);
  REQUIRE(fly_to(0, 0, 2) > 0);
  run(10, [] { return path_done() == 1; });
  run(2, [] { return false; });
}

TEST_CASE("bindings: go home lands on the pad center flying on the estimate, motors off, the clock keeps going") {
  hover_over_home();
  land(0, 0);
  run(15, [] { return is_landed() == 1; });
  const double* s = state_now();
  CHECK(!is_crashed());
  REQUIRE(is_landed());
  // on the pad center, at rest on its skids, rotors stopped
  CHECK(std::hypot(s[0], s[1]) < 0.1);
  CHECK(std::abs(s[2] - 0.115) < 1e-9);
  for (int i = 13; i < 17; i++) CHECK(s[i] == 0);
  // sitting there it doesn't move, but the sensor clock still runs so the page's plot and lidar keep their timing
  double t = sim_tick();
  advance(500);
  CHECK(sim_tick() == t + 500);
  CHECK(std::abs(state_now()[2] - 0.115) < 1e-9);
  CHECK(!is_crashed());
}

TEST_CASE("bindings: after a landing it takes off for the next goal without counting the pad as a crash") {
  hover_over_home();
  land(0, 0);
  run(15, [] { return is_landed() == 1; });
  REQUIRE(is_landed());
  REQUIRE(fly_to(-10, 10, 5) > 0);
  run(15, [] { return path_done() == 1; });
  CHECK(!is_crashed());
  CHECK(!is_landed());
  CHECK(state_now()[2] > 4);
}

TEST_CASE("bindings: a goal it can't reach, asked for during a landing, doesn't stop the landing") {
  hover_over_home();
  land(0, 0);
  run(1, [] { return false; });
  // far outside the planning grid, so there's no path
  CHECK(fly_to(200, 200, 5) == 0);
  run(15, [] { return is_landed() == 1; });
  CHECK(!is_crashed());
  CHECK(is_landed());
}

TEST_CASE("bindings: outside a landing the ground still ends the run") {
  new_city(1);
  reset(0, 0, 0.3, 0);
  advance(5);
  CHECK(is_crashed());
}
