#include "map.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>

std::vector<Box> city(uint32_t seed) {
  // mt19937 gives the same numbers on every compiler, the std distributions don't,
  // so this turns raw draws into numbers by hand to keep the browser and native maps identical
  std::mt19937 rng(seed);
  auto uniform = [&](double lo, double hi) { return lo + (hi - lo) * (rng() / 4294967296.0); };

  // 5 x 5 city blocks, 8 m lots on a 12 m grid so 4 m streets run between them
  std::vector<Box> boxes;
  for (int i = -2; i <= 2; i++)
    for (int j = -2; j <= 2; j++) {
      // the middle lot is home, kept empty for takeoff
      if (i == 0 && j == 0) continue;
      // some lots stay empty, like parks or parking
      if (uniform(0, 1) < 0.2) continue;

      // a building of random size somewhere inside its lot
      double cx = 12.0 * i, cy = 12.0 * j;
      double sx = uniform(4, 8), sy = uniform(4, 8), h = uniform(4, 18);
      double x0 = cx - 4 + uniform(0, 8 - sx), y0 = cy - 4 + uniform(0, 8 - sy);
      boxes.push_back({{x0, y0, 0}, {x0 + sx, y0 + sy, h}});
    }
  return boxes;
}

Box grow(Box b, double m) { return {b.lo - Vec3{m, m, m}, b.hi + Vec3{m, m, m}}; }

Grid::Grid(int nx, int ny, int nz, double res, Vec3 origin)
    : nx(nx), ny(ny), nz(nz), res(res), origin(origin), occ(size_t(nx) * ny * nz, 0) {}

void mark(Grid& g, const Box& b) {
  // first and last cell along one axis whose center falls inside lo..hi, clamped to the grid
  auto first = [&](double lo, double o) { return std::max(0, int(std::ceil((lo - o) / g.res - 0.5))); };
  auto last = [&](double hi, double o, int n) { return std::min(n - 1, int(std::floor((hi - o) / g.res - 0.5))); };
  for (int k = first(b.lo.z, g.origin.z); k <= last(b.hi.z, g.origin.z, g.nz); k++)
    for (int j = first(b.lo.y, g.origin.y); j <= last(b.hi.y, g.origin.y, g.ny); j++)
      for (int i = first(b.lo.x, g.origin.x); i <= last(b.hi.x, g.origin.x, g.nx); i++) g.block(i, j, k);
}

Grid city_grid(const std::vector<Box>& boxes) {
  Grid g(int(2 * kHalfWidth / kVoxel), int(2 * kHalfWidth / kVoxel), int(kCeiling / kVoxel));
  // the ground and ceiling are walls too, kept kMargin away like the buildings
  for (int k = 0; k < g.nz; k++) {
    double z = g.center(0, 0, k).z;
    if (z < kMargin || z > kCeiling - kMargin)
      for (int j = 0; j < g.ny; j++)
        for (int i = 0; i < g.nx; i++) g.block(i, j, k);
  }
  // every building, grown so the drone can be treated as a point
  for (const Box& b : boxes) mark(g, grow(b, kMargin));
  return g;
}

bool segment_hits(Vec3 a, Vec3 b, const Box& box) {
  // walk the segment as a + t (b - a) for t from 0 to 1, and squeeze that range one axis at a time
  // down to the part that's between the box's two faces on that axis
  double t0 = 0, t1 = 1;
  double from[3] = {a.x, a.y, a.z}, d[3] = {b.x - a.x, b.y - a.y, b.z - a.z};
  double lo[3] = {box.lo.x, box.lo.y, box.lo.z}, hi[3] = {box.hi.x, box.hi.y, box.hi.z};
  for (int i = 0; i < 3; i++) {
    if (d[i] == 0) {
      // running parallel to these faces: either always between them or never
      if (from[i] < lo[i] || from[i] > hi[i]) return false;
      continue;
    }
    double ta = (lo[i] - from[i]) / d[i], tb = (hi[i] - from[i]) / d[i];
    if (ta > tb) std::swap(ta, tb);
    t0 = std::max(t0, ta);
    t1 = std::min(t1, tb);
    if (t0 > t1) return false;  // the inside parts of two axes don't overlap, so it misses
  }
  return true;
}
