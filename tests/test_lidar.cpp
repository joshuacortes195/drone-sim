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
  CHECK(!add_hits(g, grown, scan(L, sn, s, s, boxes)));
  CHECK(g.occ == before);
  CHECK(grown.size() == boxes.size());

  // a 2 m cube dropped 5 m east of the drone
  Box dropped{{4, -0.75, 4.25}, {6, 1.25, 6.25}};
  auto world = boxes;
  world.push_back(dropped);
  auto hits = scan(L, sn, s, s, world);
  CHECK(add_hits(g, grown, hits));
  CHECK(grown.size() > boxes.size());
  // the cell in front of the face it sees, half the margin out, is blocked now
  CHECK(g.blocked(g.ix(4 - kMargin / 2), g.iy(0.25), g.iz(5.25)));
  // the same hits again add nothing
  CHECK(!add_hits(g, grown, hits));
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
  REQUIRE(add_hits(g, grown, scan(Lidar{}, sn, s, s, world)));
  CHECK(path_blocked(path, std::vector<Box>(grown.begin() + long(known), grown.end())));

  // planning again on what it knows now stays clear of the real box
  auto again = smooth(astar(g, start, goal), grown);
  REQUIRE(!again.empty());
  CHECK(!path_blocked(again, {grow(dropped, kReach)}));
}
