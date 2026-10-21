#include "doctest.h"
#include "wind.h"

#include <cmath>

// runs the gusts for 10,000 s at step dt and checks spread, mean and how long a gust lasts on the x axis
static void check_gusts(double dt) {
  Wind w = make_wind({3, 0, 0}, 1.5, 7);
  int n = int(std::lround(10000 / dt)), lag = int(std::lround(w.tau / dt));
  std::vector<double> g(n);
  for (int i = 0; i < n; i++) g[i] = wind_step(w, dt).x - 3;
  double sum = 0, sq = 0, cross = 0;
  for (int i = 0; i < n; i++) sum += g[i], sq += g[i] * g[i];
  for (int i = lag; i < n; i++) cross += g[i] * g[i - lag];
  double mean = sum / n, var = sq / n - mean * mean;
  CHECK(std::abs(mean) < 0.15);
  CHECK(std::abs(std::sqrt(var) - 1.5) < 0.075);
  // one time constant later a gust has faded to about e^-1 of itself
  CHECK(std::abs(cross / (n - lag) / var - std::exp(-1.0)) < 0.05);
}

TEST_CASE("gusts: spread 1.5 m/s around the breeze, fading over tau, the same at 1 ms and 10 ms steps") {
  check_gusts(0.001);
  check_gusts(0.01);
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
