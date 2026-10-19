#pragma once

#include "vecmath.h"

#include <cstdint>
#include <vector>

// a building or a dropped obstacle: an axis aligned box from its low corner to its high corner
struct Box {
  Vec3 lo, hi;
};

// the flying area: 60 x 60 m around home, ground to a 20 m ceiling
constexpr double kHalfWidth = 30, kCeiling = 20;

// seeded city of buildings, same seed always gives the same city
std::vector<Box> city(uint32_t seed);

// planning cells are 0.5 m cubes
constexpr double kVoxel = 0.5;
// how far the drone reaches from its center: 0.256 m arm to the farthest rotor plus the 0.1 m rotor
constexpr double kReach = 0.256 + 0.1;
// buildings grow by this before voxelizing: the drone's reach plus half a cell diagonal,
// since a cell only counts as free by its center
constexpr double kMargin = kReach + kVoxel * 0.8660254037844386;

// the same box made bigger by m on every side
Box grow(Box b, double m);

// the world cut into cells, each one free or blocked
struct Grid {
  int nx, ny, nz;
  double res;
  Vec3 origin;  // low corner of cell 0, 0, 0
  std::vector<uint8_t> occ;

  Grid(int nx, int ny, int nz, double res = kVoxel, Vec3 origin = {-kHalfWidth, -kHalfWidth, 0});
  bool inside(int i, int j, int k) const { return i >= 0 && j >= 0 && k >= 0 && i < nx && j < ny && k < nz; }
  // anything off the grid counts as blocked so paths stay on the map
  bool blocked(int i, int j, int k) const { return !inside(i, j, k) || occ[(size_t(k) * ny + j) * nx + i]; }
  void block(int i, int j, int k) { occ[(size_t(k) * ny + j) * nx + i] = 1; }
  Vec3 center(int i, int j, int k) const { return origin + Vec3{i + 0.5, j + 0.5, k + 0.5} * res; }
  // which cell a world point falls in
  int ix(double x) const { return int(std::floor((x - origin.x) / res)); }
  int iy(double y) const { return int(std::floor((y - origin.y) / res)); }
  int iz(double z) const { return int(std::floor((z - origin.z) / res)); }
};

// blocks every cell whose center is inside the box
void mark(Grid& g, const Box& b);

// the 60 x 60 x 20 m planning grid: buildings grown by kMargin, plus the ground and ceiling
Grid city_grid(const std::vector<Box>& boxes);

// true when the straight segment from a to b touches the box (slab test)
bool segment_hits(Vec3 a, Vec3 b, const Box& box);

// how far along a unit direction a ray goes before it enters the box, infinity if it doesn't within range.
// 0 when it starts inside
double ray_box(Vec3 from, Vec3 dir, double range, const Box& box);
