#include "lidar.h"
#include "planner.h"
#include "route.h"

#include <cmath>
#include <limits>

std::vector<Vec3> scan(const Lidar& L, Sensors& sn, const QuadState& truth, const QuadState& est,
                       const std::vector<Box>& world) {
  std::vector<Vec3> hits;
  for (int a = 0; a < L.steps; a++) {
    double az = 2 * 3.14159265358979323846 * a / L.steps;
    for (int b = 0; b < L.beams; b++) {
      // the beam's direction on the drone, then in the world
      double el = L.lowest + b * L.spacing;
      Vec3 dir = {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
      Vec3 out = rotate(truth.q, dir);

      // the closest box this beam runs into
      double r = std::numeric_limits<double>::infinity();
      for (const Box& box : world) r = std::min(r, ray_box(truth.p, out, L.range, box));
      if (std::isinf(r)) continue;

      // a little range noise, then put the hit where the drone thinks the beam went
      r += sn.n.lidar * gauss(sn.rng);
      hits.push_back(est.p + rotate(est.q, dir) * r);
    }
  }
  return hits;
}

bool add_hits(Grid& g, std::vector<Box>& grown, const std::vector<Vec3>& hits, const std::vector<Box>& known,
              Vec3 from) {
  bool added = false;
  for (Vec3 h : hits) {
    // off the map, or a cell that already has a hit: nothing new
    int i = g.ix(h.x), j = g.iy(h.y), k = g.iz(h.z);
    if (!g.inside(i, j, k) || g.has_hit(i, j, k)) continue;
    // on a building it already knows about, give or take how far a small heading error moves it
    double tol = kKnownTol + kKnownAngle * norm(h - from);
    bool on_known = false;
    for (const Box& b : known) on_known = on_known || segment_hits(h, h, grow(b, tol));
    if (on_known) continue;

    // grow the whole cell like a building, so it covers the margin of any hit that lands in it
    Vec3 half{g.res / 2, g.res / 2, g.res / 2};
    Box b = grow({g.center(i, j, k) - half, g.center(i, j, k) + half}, kMargin);
    mark(g, b);
    g.set_hit(i, j, k);
    grown.push_back(b);
    added = true;
  }
  return added;
}

bool replan(std::vector<Vec3>& path, const Grid& g, const std::vector<Box>& grown, size_t new_from, Vec3 from,
            Vec3 goal, Vec3* vel) {
  // only the new boxes matter, the old ones were already planned around
  if (!path_blocked(path, std::vector<Box>(grown.begin() + long(new_from), grown.end()))) return false;
  // moving: plan from where braking straight ahead ends, if that stays inside the flying area and the braking leg
  // keeps the same margin from everything known that every planned leg gets (clear of the grown boxes and the floor)
  Vec3 start = from;
  bool braking = false;
  if (vel && norm(*vel) > 0) {
    Vec3 stop = from + *vel * (norm(*vel) / (2 * kBrake));
    braking = g.inside(g.ix(stop.x), g.iy(stop.y), g.iz(stop.z)) && stop.z > kMargin && !path_blocked({from, stop}, grown);
    if (braking) start = stop;
  }
  // a fresh path on everything it knows now
  path = astar(g, start, goal);
  // no way on from where it would stop: try from where it is, flown from rest like before
  if (braking && path.empty()) {
    braking = false;
    start = from;
    path = astar(g, start, goal);
  }
  // no braking leg, or nowhere to go: the caller flies it from rest
  if (vel && (!braking || path.empty())) *vel = {};
  // start it right where it'll be instead of its cell center, so the setpoint doesn't jump. any point in a free cell
  // is clear of the walls by the margin. a blocked start keeps the free cell it found, after the braking leg if any
  if (!path.empty()) {
    if (!g.blocked(g.ix(start.x), g.iy(start.y), g.iz(start.z))) path[0] = start;
    else if (braking) path.insert(path.begin(), start);
  }
  path = smooth(path, grown);
  // the braking leg goes first, from the drone to where it stops, so the next scan checks it like any other leg
  if (braking && !path.empty()) path.insert(path.begin(), from);
  return true;
}
