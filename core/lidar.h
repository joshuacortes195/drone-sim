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

// how close a hit has to be to a known building to count as that building: 0.1 m, over 3x the lidar's 3 cm
// noise, plus 0.03 m per meter away, since a heading error moves a hit sideways by range times the angle.
// 0.03 rad is just over the worst attitude error measured on the rtk flights (0.027 rad, results/lidar.md)
constexpr double kKnownTol = 0.1, kKnownAngle = 0.03;

// adds what the lidar saw to what the drone knows. hits on known buildings are skipped, and so are hits in a
// cell that already has one. every other hit's cell gets grown by the drone's margin, blocked on the grid and
// added to grown as a box for smoothing and replanning. from is where the drone thinks it is.
// true when anything new turned up
bool add_hits(Grid& g, std::vector<Box>& grown, const std::vector<Vec3>& hits, const std::vector<Box>& known,
              Vec3 from);
