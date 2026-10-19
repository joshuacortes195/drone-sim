#include "doctest.h"
#include "lidar.h"
#include "planner.h"

#include <algorithm>
#include <cmath>

TEST_CASE("ray_box: distance to the face it goes in through, misses, parallel, inside and out of range") {
  Box b{{2, -1, -1}, {4, 1, 1}};
  // straight down the x axis into the near face
  CHECK(std::abs(ray_box({0, 0, 0}, {1, 0, 0}, 100, b) - 2) < 1e-12);
  // 45 degrees up from below the box: crosses x = 2 at z = 0.5, so it goes in through the near face
  double s = std::sqrt(0.5);
  CHECK(std::abs(ray_box({0, 0, -1.5}, {s, 0, s}, 100, b) - 2 * std::sqrt(2.0)) < 1e-12);
  // 45 degrees up from further down: still under the box at x = 2, goes in through the bottom at x = 2.5
  CHECK(std::abs(ray_box({0, 0, -3.5}, {s, 0, s}, 100, b) - 2.5 * std::sqrt(2.0)) < 1e-12);
  CHECK(std::isinf(ray_box({0, 0, 0}, {-1, 0, 0}, 100, b)));  // pointing away
  CHECK(std::isinf(ray_box({0, 0, 2}, {1, 0, 0}, 100, b)));   // parallel, over the top
  CHECK(ray_box({3, 0, 0}, {1, 0, 0}, 100, b) == 0);          // starting inside
  CHECK(std::isinf(ray_box({0, 0, 0}, {1, 0, 0}, 1.5, b)));   // the face is past the range
  CHECK(std::abs(ray_box({0, 0, 0}, {1, 0, 0}, 2, b) - 2) < 1e-12);  // right at the range still counts
}

// a drone sitting level and facing east at p
static QuadState at(Vec3 p) {
  QuadState s{};
  s.p = p;
  s.q = {1, 0, 0, 0};
  return s;
}

TEST_CASE("scan: hits land on the nearest box in the vlp-16 pattern, a box hidden behind it gets none") {
  Lidar L;
  Sensors sn = make_sensors(quiet(), 1);
  QuadState s = at({0, 0, 5});
  // a wall 5 m ahead, 6 m wide and 10 m tall, and a smaller box hidden right behind it
  Box wall{{5, -3, 0}, {6, 3, 10}}, hidden{{8, -1, 0}, {9, 1, 10}};
  auto hits = scan(L, sn, s, s, {wall, hidden});
  // every hit is on the wall's near face, none on the hidden box
  for (Vec3 h : hits) CHECK(std::abs(h.x - 5) < 1e-9);
  // the face spans 5 tan(az) <= 3, so the 31 headings from -30 to +30 deg, times 16 beams
  CHECK(hits.size() == 31 * 16);
  // straight ahead the top beam is 15 deg up and the bottom one 15 deg down
  double top = -1, bottom = 1;
  for (Vec3 h : hits)
    if (std::abs(h.y) < 1e-9) top = std::max(top, h.z - 5), bottom = std::min(bottom, h.z - 5);
  CHECK(std::abs(top - 5 * std::tan(15 * 3.14159265358979323846 / 180)) < 1e-9);
  CHECK(std::abs(bottom + top) < 1e-9);
}

TEST_CASE("scan: ranges come from where the drone really is, hits get placed where it thinks it is") {
  Lidar L;
  Sensors sn = make_sensors(quiet(), 1);
  std::vector<Box> world = {{{5, -3, 0}, {6, 3, 10}}, {{-2, 4, 0}, {2, 6, 8}}};
  QuadState truth = at({0, 0, 5});
  // the estimate is 0.5 m off to the east and turned 90 deg to the left
  QuadState est = truth;
  est.p = {0.5, 0, 5};
  est.q = axis_angle({0, 0, 1}, 3.14159265358979323846 / 2);
  auto real = scan(L, sn, truth, truth, world);
  auto placed = scan(L, sn, truth, est, world);
  REQUIRE(real.size() == placed.size());
  REQUIRE(!real.empty());
  // each hit is the true one moved by the estimate's error: turned 90 deg about the drone, then shifted
  for (size_t i = 0; i < real.size(); i++) {
    Vec3 want = est.p + rotate(est.q, real[i] - truth.p);
    CHECK(norm(placed[i] - want) < 1e-9);
  }
}

TEST_CASE("scan: range noise has the vlp-16's 3 cm spread and no offset") {
  Lidar L;
  Sensors sn = make_sensors(Noise{}, 7);
  QuadState s = at({0, 0, 5});
  Box wall{{5, -3, 0}, {6, 3, 10}};
  // how far each hit is from the wall along its own ray
  double sum = 0, sum2 = 0;
  int n = 0;
  for (int scans = 0; scans < 10; scans++)
    for (Vec3 h : scan(L, sn, s, s, {wall})) {
      Vec3 d = h - s.p;
      double r = norm(d), r_true = 5 * r / d.x;
      sum += r - r_true;
      sum2 += (r - r_true) * (r - r_true);
      n++;
    }
  double mean = sum / n, sd = std::sqrt(sum2 / n - mean * mean);
  CHECK(n == 10 * 31 * 16);
  CHECK(std::abs(mean) < 0.003);
  CHECK(std::abs(sd - 0.03) < 0.003);
}

TEST_CASE("add_hits: known buildings add nothing, a dropped box gets blocked once") {
  auto boxes = city(3);
  Grid g = city_grid(boxes);
  std::vector<Box> grown;
  for (const Box& b : boxes) grown.push_back(grow(b, kMargin));
  Sensors sn = make_sensors(quiet(), 1);
  QuadState s = at({0.25, 0.25, 5.25});
  Lidar L;

  // only the city it already knows: nothing new
  auto before = g.occ;
  REQUIRE(!scan(L, sn, s, s, boxes).empty());
  CHECK(!add_hits(g, grown, scan(L, sn, s, s, boxes), boxes, s.p));
  CHECK(g.occ == before);
  CHECK(grown.size() == boxes.size());

  // a 2 m cube dropped 5 m east of the drone
  Box dropped{{4, -0.75, 4.25}, {6, 1.25, 6.25}};
  auto world = boxes;
  world.push_back(dropped);
  auto hits = scan(L, sn, s, s, world);
  CHECK(add_hits(g, grown, hits, boxes, s.p));
  CHECK(grown.size() > boxes.size());
  // the cell in front of the face it sees, half the margin out, is blocked now
  CHECK(g.blocked(g.ix(4 - kMargin / 2), g.iy(0.25), g.iz(5.25)));
  // the same hits again add nothing
  CHECK(!add_hits(g, grown, hits, boxes, s.p));
}

TEST_CASE("replan: a dropped box the lidar sees cuts the path, the new path goes around it") {
  auto boxes = city(3);
  Grid g = city_grid(boxes);
  std::vector<Box> grown;
  for (const Box& b : boxes) grown.push_back(grow(b, kMargin));
  Vec3 start = g.center(g.ix(0.25), g.iy(0.25), g.iz(5.25));
  Vec3 goal = g.center(g.ix(18.25), g.iy(18.25), g.iz(5.25));
  auto path = smooth(astar(g, start, goal), grown);
  REQUIRE(path.size() >= 2);

  // a 2 m cube on the middle of the first leg, which the drone doesn't know about
  REQUIRE(norm(path[1] - path[0]) > 6);
  Vec3 mid = (path[0] + path[1]) * 0.5;
  Box dropped{mid - Vec3{1, 1, 1}, mid + Vec3{1, 1, 1}};
  auto world = boxes;
  world.push_back(dropped);

  // a scan from the start sees it, and the new cells cut the path
  Sensors sn = make_sensors(rtk(), 5);
  QuadState s = at(start);
  size_t known = grown.size();
  REQUIRE(add_hits(g, grown, scan(Lidar{}, sn, s, s, world), boxes, s.p));
  CHECK(path_blocked(path, std::vector<Box>(grown.begin() + long(known), grown.end())));

  // planning again on what it knows now stays clear of the real box
  auto again = smooth(astar(g, start, goal), grown);
  REQUIRE(!again.empty());
  CHECK(!path_blocked(again, {grow(dropped, kReach)}));
}

// distance from a point to the outside of a box, 0 when inside
static double box_dist(Vec3 p, const Box& b) {
  double dx = std::max({b.lo.x - p.x, 0.0, p.x - b.hi.x});
  double dy = std::max({b.lo.y - p.y, 0.0, p.y - b.hi.y});
  double dz = std::max({b.lo.z - p.z, 0.0, p.z - b.hi.z});
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

TEST_CASE("add_hits: every cell close to the face it saw gets blocked, even where the hits crowd together") {
  Grid g = city_grid({});
  std::vector<Box> grown;
  Sensors sn = make_sensors(quiet(), 1);
  QuadState s = at({0.25, 0.25, 5.25});
  // a thin plate 5 m east, 4 m wide and 2 m tall, all inside the beams' 15 deg up and down
  Box plate{{5, -2, 4.25}, {5.1, 2, 6.25}};
  Box face{{5, -2, 4.25}, {5, 2, 6.25}};
  CHECK(add_hits(g, grown, scan(Lidar{}, sn, s, s, {plate}), {}, s.p));
  // every spot nearly the drone's margin from that face (less 0.15 m for the gaps between hits) has to be
  // blocked on the grid for a*, and inside one of the grown boxes for smoothing, checked on a 0.1 m lattice
  int near = 0, off_grid = 0, off_boxes = 0;
  for (double z = 3; z < 7.5; z += 0.1)
    for (double y = -3.5; y < 3.5; y += 0.1)
      for (double x = 3.5; x < 5; x += 0.1) {
        Vec3 p{x, y, z};
        if (box_dist(p, face) >= kMargin - 0.15) continue;
        near++;
        if (!g.blocked(g.ix(x), g.iy(y), g.iz(z))) off_grid++;
        bool inside = false;
        for (const Box& b : grown) inside = inside || box_dist(p, b) == 0;
        if (!inside) off_boxes++;
      }
  CHECK(near > 1000);
  CHECK(off_grid == 0);
  CHECK(off_boxes == 0);
}

TEST_CASE("add_hits: the side of a box seen after its front still gets its own boxes, so a path along it gets cut") {
  Grid g = city_grid({});
  std::vector<Box> grown;
  Sensors sn = make_sensors(quiet(), 1);
  Box cube{{4, -1, 4.25}, {6, 1, 6.25}};
  // first from straight in front, then from 1.5 m off its south side
  add_hits(g, grown, scan(Lidar{}, sn, at({0.25, 0.25, 5.25}), at({0.25, 0.25, 5.25}), {cube}), {}, Vec3{0.25, 0.25, 5.25});
  size_t before = grown.size();
  QuadState side = at({5, -2.5, 5.25});
  CHECK(add_hits(g, grown, scan(Lidar{}, sn, side, side, {cube}), {}, side.p));
  std::vector<Box> fresh(grown.begin() + long(before), grown.end());
  // a path 0.2 m off the south face only shows up as blocked against the new boxes
  CHECK(path_blocked({{3.5, -1.2, 5.25}, {6.5, -1.2, 5.25}}, fresh));

  // the strip of the south face the beams reach from there, 15 deg up and down over 1.5 m
  Box strip{{4, -1, 5.25 - 0.35}, {6, -1, 5.25 + 0.35}};
  int near = 0, off_boxes = 0;
  for (double z = 4; z < 6.5; z += 0.1)
    for (double y = -2; y < -1; y += 0.1)
      for (double x = 3; x < 7; x += 0.1) {
        Vec3 p{x, y, z};
        if (box_dist(p, strip) >= kMargin - 0.15) continue;
        near++;
        bool inside = false;
        for (const Box& b : grown) inside = inside || box_dist(p, b) == 0;
        if (!inside) off_boxes++;
      }
  CHECK(near > 500);
  CHECK(off_boxes == 0);
}

TEST_CASE("add_hits: a heading error smears far hits off known buildings, they still count as known") {
  auto boxes = city(3);
  Grid g = city_grid(boxes);
  std::vector<Box> grown;
  for (const Box& b : boxes) grown.push_back(grow(b, kMargin));
  Sensors sn = make_sensors(quiet(), 1);
  QuadState truth = at({0.25, 0.25, 5.25});
  // the estimate's heading is 0.025 rad off, about the worst the rtk flights had
  QuadState est = truth;
  est.q = axis_angle({0, 0, 1}, 0.025);
  auto hits = scan(Lidar{}, sn, truth, est, boxes);
  // some hits really are pushed well off their walls
  double off = 0;
  for (Vec3 h : hits) {
    double d = 1e9;
    for (const Box& b : boxes) d = std::min(d, box_dist(h, b));
    off = std::max(off, d);
  }
  CHECK(off > 0.3);
  auto before = g.occ;
  CHECK(!add_hits(g, grown, hits, boxes, est.p));
  CHECK(g.occ == before);
}

TEST_CASE("add_hits: a new box about a meter off a known building, seen from close, is not taken for the building") {
  Grid g = city_grid({});
  std::vector<Box> grown;
  Sensors sn = make_sensors(quiet(), 1);
  QuadState s = at({0.25, 0.25, 5.25});
  // a known building 5 m east, and a small new box in front of its corner, the face the drone sees
  // 1.1 to 1.4 m from the building. the tolerance at 4 m is 0.22 m, so all of it is new
  Box known{{5, 1, 0}, {7, 4, 10}}, fresh{{4, 0, 4.75}, {4.5, 0.5, 5.75}};
  CHECK(add_hits(g, grown, scan(Lidar{}, sn, s, s, {known, fresh}), {known}, s.p));
}
