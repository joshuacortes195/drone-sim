#pragma once

#include "vecmath.h"

#include <random>

// the air around the drone: a steady breeze plus gusts that come and go.
// each gust axis is a first order gauss markov process (filtered noise), the same one pole shape dryden uses along the wind
struct Wind {
  Vec3 breeze;       // m/s, the steady part
  double sigma = 0;  // m/s, how big the gusts are on each axis
  double tau = 2;    // s, about how long a gust lasts. my pick so gusts are visible, not a dryden length scale
  Vec3 gust;         // the gust right now
  std::mt19937 rng;
};

// a calm start with this breeze and gust strength, the same seed always gives the same gusts
Wind make_wind(Vec3 breeze, double sigma, unsigned seed);

// moves the gusts on dt seconds and returns the wind the drone feels now
Vec3 wind_step(Wind& w, double dt);
