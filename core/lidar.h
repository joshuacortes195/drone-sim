#pragma once

#include "map.h"
#include "sensors.h"

#include <vector>

// velodyne vlp-16 (data sheet 63-9229 rev k): 16 beams from -15 to +15 deg, 2 deg apart, 360 deg around,
// 100 m, spun at 10 Hz. at 10 Hz the real one steps 0.2 deg around, this steps 2 deg to keep it cheap
struct Lidar {
  int beams = 16;
  double lowest = -15 * 3.14159265358979323846 / 180, spacing = 2 * 3.14159265358979323846 / 180;
  int steps = 180;
  double range = 100;
  double period = 0.1;
};

// one full turn of the lidar, taken all at once. distances come from where the drone really is (truth),
// the hits get placed in the world from where it thinks it is (est). only boxes reflect, the ground is known
std::vector<Vec3> scan(const Lidar& L, Sensors& sn, const QuadState& truth, const QuadState& est,
                       const std::vector<Box>& world);

// blocks the cells around every hit that isn't already blocked, and adds each one to grown as a box
// for smoothing and replanning. true when anything new turned up
bool add_hits(Grid& g, std::vector<Box>& grown, const std::vector<Vec3>& hits);
