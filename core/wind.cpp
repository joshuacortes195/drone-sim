#include "wind.h"

#include "sensors.h"

#include <cmath>

Wind make_wind(Vec3 breeze, double sigma, unsigned seed) {
  Wind w;
  w.breeze = breeze;
  w.sigma = sigma;
  w.rng.seed(seed);
  return w;
}

Vec3 wind_step(Wind& w, double dt) {
  if (w.sigma == 0) return w.breeze;
  // each gust fades toward zero and gets a random kick, sized so the spread stays sigma at any step size
  double a = std::exp(-dt / w.tau), kick = w.sigma * std::sqrt(1 - a * a);
  w.gust.x = a * w.gust.x + kick * gauss(w.rng);
  w.gust.y = a * w.gust.y + kick * gauss(w.rng);
  w.gust.z = a * w.gust.z + kick * gauss(w.rng);
  return w.breeze + w.gust;
}
