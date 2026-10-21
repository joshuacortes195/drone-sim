#include "doctest.h"
#include "wind.h"

#include <cmath>

// runs the gusts for 10,000 s at step dt and checks spread, mean and how long a gust lasts on every axis,
// and that the axes don't move together
static void check_gusts(double dt) {
  Wind w = make_wind({3, -1, 0}, 1.5, 7);
  int n = int(std::lround(10000 / dt)), lag = int(std::lround(w.tau / dt));
  std::vector<double> g[3];
  for (int i = 0; i < n; i++) {
    Vec3 v = wind_step(w, dt);
    g[0].push_back(v.x - 3), g[1].push_back(v.y + 1), g[2].push_back(v.z);
  }
  for (int a = 0; a < 3; a++) {
    double sum = 0, sq = 0, cross = 0;
    for (int i = 0; i < n; i++) sum += g[a][i], sq += g[a][i] * g[a][i];
    for (int i = lag; i < n; i++) cross += g[a][i] * g[a][i - lag];
    double mean = sum / n, var = sq / n - mean * mean;
    INFO("axis ", a, " dt ", dt);
    CHECK(std::abs(mean) < 0.15);
    CHECK(std::abs(std::sqrt(var) - 1.5) < 0.075);
    // one time constant later a gust has faded to about e^-1 of itself
    CHECK(std::abs(cross / (n - lag) / var - std::exp(-1.0)) < 0.05);
  }
  // each axis gets its own random kicks, so x and y barely line up
  double xy = 0;
  for (int i = 0; i < n; i++) xy += g[0][i] * g[1][i];
  CHECK(std::abs(xy / n / (1.5 * 1.5)) < 0.05);
}

TEST_CASE("gusts: spread 1.5 m/s around the breeze, fading over tau, the same at 1 ms, 10 ms and 1 s steps") {
  check_gusts(0.001);
  check_gusts(0.01);
  // at a step half as long as tau a simple euler update would spread 1.89 m/s, only the exact one stays at 1.5
  check_gusts(1.0);
}

TEST_CASE("gusts: no gust strength gives exactly the breeze") {
  Wind w = make_wind({2, -1, 0}, 0, 3);
  for (int i = 0; i < 100; i++) {
    Vec3 v = wind_step(w, 0.001);
    CHECK(v.x == 2);
    CHECK(v.y == -1);
    CHECK(v.z == 0);
  }
}
