#include "doctest.h"
#include "map.h"
#include "planner.h"

#include <algorithm>
#include <cmath>
#include <random>

// true when two cities have exactly the same boxes
static bool same(const std::vector<Box>& a, const std::vector<Box>& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); i++)
    if (a[i].lo.x != b[i].lo.x || a[i].lo.y != b[i].lo.y || a[i].lo.z != b[i].lo.z ||
        a[i].hi.x != b[i].hi.x || a[i].hi.y != b[i].hi.y || a[i].hi.z != b[i].hi.z)
      return false;
  return true;
}

TEST_CASE("city: same seed same map, different seed different map") {
  CHECK(!city(1).empty());
  CHECK(same(city(1), city(1)));
  CHECK(!same(city(1), city(2)));
}

TEST_CASE("city: every building sits on the ground, inside the area, under the ceiling, off the home spot") {
  for (uint32_t seed = 0; seed < 100; seed++)
    for (const Box& b : city(seed)) {
      CHECK(b.lo.z == 0);
      CHECK(b.hi.z < kCeiling - 1);  // leaves room to fly over the tallest one
      CHECK(b.lo.x >= -kHalfWidth);
      CHECK(b.lo.y >= -kHalfWidth);
      CHECK(b.hi.x <= kHalfWidth);
      CHECK(b.hi.y <= kHalfWidth);
      CHECK(b.lo.x < b.hi.x);
      CHECK(b.lo.y < b.hi.y);
      // home at the origin stays an open square so the drone can take off
      bool covers_home = b.lo.x < 4 && b.hi.x > -4 && b.lo.y < 4 && b.hi.y > -4;
      CHECK(!covers_home);
    }
}

// how many cells are blocked in the whole grid
static int count_blocked(const Grid& g) {
  int n = 0;
  for (uint8_t c : g.occ) n += c;
  return n;
}

TEST_CASE("mark: a box blocks exactly the cells whose centers it holds") {
  // 1 m cells from the origin, so cell i has its center at i + 0.5
  Grid g(10, 10, 10, 1.0, {0, 0, 0});
  mark(g, {{2.2, 3.0, 0}, {4.6, 5.5, 1.4}});
  // x centers 2.5 3.5 4.5, y centers 3.5 4.5 5.5 (on the edge counts), z center 0.5: 9 cells
  CHECK(count_blocked(g) == 9);
  CHECK(g.blocked(2, 3, 0));
  CHECK(g.blocked(4, 5, 0));
  CHECK(!g.blocked(1, 3, 0));
  CHECK(!g.blocked(5, 4, 0));
  CHECK(!g.blocked(3, 2, 0));
  CHECK(!g.blocked(3, 4, 1));
  // off the grid is a wall
  CHECK(g.blocked(-1, 0, 0));
  CHECK(g.blocked(0, 10, 0));
}

TEST_CASE("city grid: buildings grow by the drone's reach plus half a cell, ground and ceiling are walls") {
  Grid g = city_grid({{{5, 5, 0}, {9, 9, 10}}});
  CHECK(g.nx == 120);
  CHECK(g.ny == 120);
  CHECK(g.nz == 40);
  // cell index for a world coordinate on these 0.5 m cells starting at -30, -30, 0
  auto ix = [](double x) { return int((x + 30) / 0.5); };
  auto iz = [](double z) { return int(z / 0.5); };
  // margin is 0.256 + 0.1 + 0.433 = 0.789 m
  CHECK(g.blocked(ix(7.25), ix(7.25), iz(5.25)));    // inside the building
  CHECK(g.blocked(ix(9.75), ix(7.25), iz(5.25)));    // 0.75 m off the east wall
  CHECK(!g.blocked(ix(10.25), ix(7.25), iz(5.25)));  // 1.25 m off it
  CHECK(g.blocked(ix(7.25), ix(7.25), iz(10.75)));   // 0.75 m above the roof
  CHECK(!g.blocked(ix(7.25), ix(7.25), iz(11.25)));
  // ground: 0.75 m up is too low, 1.25 m is fine. ceiling: 19.25 is too high, 18.75 is fine
  CHECK(g.blocked(ix(0.25), ix(0.25), iz(0.75)));
  CHECK(!g.blocked(ix(0.25), ix(0.25), iz(1.25)));
  CHECK(g.blocked(ix(0.25), ix(0.25), iz(19.25)));
  CHECK(!g.blocked(ix(0.25), ix(0.25), iz(18.75)));
}

// total length of a path
static double length(const std::vector<Vec3>& path) {
  double L = 0;
  for (size_t i = 1; i < path.size(); i++) L += norm(path[i] - path[i - 1]);
  return L;
}

// true when every step moves to a touching cell and every cell on the way is free
static bool walkable(const Grid& g, const std::vector<Vec3>& path) {
  for (size_t n = 0; n < path.size(); n++) {
    Vec3 p = path[n];
    if (g.blocked(g.ix(p.x), g.iy(p.y), g.iz(p.z))) return false;
    if (n > 0) {
      Vec3 d = p - path[n - 1];
      if (std::abs(d.x) > g.res * 1.01 || std::abs(d.y) > g.res * 1.01 || std::abs(d.z) > g.res * 1.01) return false;
    }
  }
  return true;
}

// a small empty grid of 0.5 m cells from the origin, for hand built tests
static Grid open_grid(int nx, int ny, int nz) { return Grid(nx, ny, nz, 0.5, {0, 0, 0}); }

TEST_CASE("astar: open grid, straight across the cube diagonal") {
  Grid g = open_grid(10, 10, 10);
  auto path = astar(g, g.center(0, 0, 0), g.center(9, 9, 9));
  REQUIRE(!path.empty());
  CHECK(walkable(g, path));
  CHECK(norm(path.front() - g.center(0, 0, 0)) < 1e-12);
  CHECK(norm(path.back() - g.center(9, 9, 9)) < 1e-12);
  // 9 corner steps of 0.5 * sqrt(3)
  CHECK(length(path) == doctest::Approx(7.794228634));
}

TEST_CASE("astar: open grid, mix of diagonal and straight steps") {
  Grid g = open_grid(10, 10, 10);
  auto path = astar(g, g.center(0, 0, 0), g.center(9, 4, 0));
  REQUIRE(!path.empty());
  CHECK(walkable(g, path));
  // 4 flat diagonals of 0.5 * sqrt(2) plus 5 straight steps of 0.5
  CHECK(length(path) == doctest::Approx(5.328427125));
}

TEST_CASE("astar: goes around a wall through the gap without cutting its corner") {
  // one layer, a wall at i = 5 from j = 0 to 8, the only gap is at j = 9
  Grid g = open_grid(10, 10, 1);
  for (int j = 0; j <= 8; j++) g.block(5, j, 0);
  auto path = astar(g, g.center(0, 0, 0), g.center(9, 0, 0));
  REQUIRE(!path.empty());
  CHECK(walkable(g, path));
  // worked by hand: (0,0) to (4,9) is 4 diagonals + 5 straight, through the gap (4,9) (5,9) (6,9) is 2 straight,
  // (6,9) to (9,0) is 3 diagonals + 6 straight. sliding past the wall's end diagonally isn't allowed.
  // 7 * 0.5 * sqrt(2) + 13 * 0.5
  CHECK(length(path) == doctest::Approx(11.449747468));
}

TEST_CASE("astar: still the shortest when the gap is off to the side and heading straight at the goal is a trap") {
  // one layer, a wall at i = 5 from j = 1 to 9, the only gap is at the bottom, j = 0
  Grid g = open_grid(10, 10, 1);
  for (int j = 1; j <= 9; j++) g.block(5, j, 0);
  auto path = astar(g, g.center(0, 5, 0), g.center(9, 5, 0));
  REQUIRE(!path.empty());
  CHECK(walkable(g, path));
  // worked by hand: (0,5) to (4,0) is 4 diagonals + 1 straight, through the gap (4,0) (5,0) (6,0) is 2 straight,
  // (6,0) to (9,5) is 3 diagonals + 2 straight. 7 * 0.5 * sqrt(2) + 5 * 0.5
  CHECK(length(path) == doctest::Approx(7.449747468));
}

// shortest distance between two cells the slow, obvious way: dijkstra with no heuristic, scanning every cell
// each round. same moves as the planner (26 neighbors, no squeezing past a blocked corner). -1 when unreachable
static double dijkstra(const Grid& g, int si, int sj, int sk, int ti, int tj, int tk) {
  int n = g.nx * g.ny * g.nz;
  std::vector<double> d(n, 1e18);
  std::vector<bool> done(n, false);
  d[(sk * g.ny + sj) * g.nx + si] = 0;
  while (true) {
    int c = -1;
    for (int x = 0; x < n; x++)
      if (!done[x] && d[x] < 1e17 && (c < 0 || d[x] < d[c])) c = x;
    if (c < 0) return -1;
    int ci = c % g.nx, cj = (c / g.nx) % g.ny, ck = c / (g.nx * g.ny);
    if (ci == ti && cj == tj && ck == tk) return d[c];
    done[c] = true;
    for (int dk = -1; dk <= 1; dk++)
      for (int dj = -1; dj <= 1; dj++)
        for (int di = -1; di <= 1; di++) {
          if (!di && !dj && !dk) continue;
          bool ok = true;
          for (int a : {0, di})
            for (int b : {0, dj})
              for (int e : {0, dk}) ok = ok && !g.blocked(ci + a, cj + b, ck + e);
          if (!ok) continue;
          int m = ((ck + dk) * g.ny + cj + dj) * g.nx + ci + di;
          d[m] = std::min(d[m], d[c] + g.res * std::sqrt(double(di * di + dj * dj + dk * dk)));
        }
  }
}

TEST_CASE("astar: same length as plain dijkstra on random cluttered grids") {
  std::mt19937 rng(11);
  int compared = 0;
  for (int trial = 0; trial < 40; trial++) {
    // a 12 x 12 x 3 grid with about a third of the cells blocked, start and goal at opposite corners
    Grid g = open_grid(12, 12, 3);
    for (int k = 0; k < 3; k++)
      for (int j = 0; j < 12; j++)
        for (int i = 0; i < 12; i++)
          if (rng() % 3 == 0) g.block(i, j, k);
    g.occ[0] = 0;
    g.occ[g.occ.size() - 1] = 0;
    double best = dijkstra(g, 0, 0, 0, 11, 11, 2);
    auto path = astar(g, g.center(0, 0, 0), g.center(11, 11, 2));
    CHECK(path.empty() == (best < 0));
    if (best < 0) continue;
    compared++;
    CHECK(walkable(g, path));
    CHECK(length(path) == doctest::Approx(best));
  }
  CHECK(compared >= 20);
}

TEST_CASE("astar: no path when the goal is walled off or blocked") {
  Grid g = open_grid(10, 10, 10);
  // a closed shell around cell 7, 7, 7
  for (int i = 6; i <= 8; i++)
    for (int j = 6; j <= 8; j++)
      for (int k = 6; k <= 8; k++)
        if (i != 7 || j != 7 || k != 7) g.block(i, j, k);
  CHECK(astar(g, g.center(0, 0, 0), g.center(7, 7, 7)).empty());
  CHECK(astar(g, g.center(0, 0, 0), g.center(6, 6, 6)).empty());
}

TEST_CASE("astar: a blocked start plans from the nearest free cell, but not from deep inside a wall") {
  Grid g = open_grid(12, 12, 12);
  // a solid 3 x 3 x 3 block around cell 5, 5, 5
  for (int i = 4; i <= 6; i++)
    for (int j = 4; j <= 6; j++)
      for (int k = 4; k <= 6; k++) g.block(i, j, k);
  auto path = astar(g, g.center(5, 5, 5), g.center(11, 11, 11));
  REQUIRE(!path.empty());
  // the closest free cells are the face neighbors two cells out, 1 m away
  CHECK(!g.blocked(g.ix(path[0].x), g.iy(path[0].y), g.iz(path[0].z)));
  CHECK(std::abs(norm(path[0] - g.center(5, 5, 5)) - 1.0) < 1e-9);

  // 4 cells out (2 m) is the farthest it still looks
  Grid four = open_grid(20, 20, 20);
  for (int i = 7; i <= 13; i++)
    for (int j = 7; j <= 13; j++)
      for (int k = 7; k <= 13; k++) four.block(i, j, k);
  auto out = astar(four, four.center(10, 10, 10), four.center(0, 0, 0));
  REQUIRE(!out.empty());
  CHECK(std::abs(norm(out[0] - four.center(10, 10, 10)) - 2.0) < 1e-9);

  // the closest free cell is 5 cells out, one past the 4 it looks, so no path
  Grid deep = open_grid(20, 20, 20);
  for (int i = 6; i <= 14; i++)
    for (int j = 6; j <= 14; j++)
      for (int k = 6; k <= 14; k++) deep.block(i, j, k);
  CHECK(astar(deep, deep.center(10, 10, 10), deep.center(0, 0, 0)).empty());
}

TEST_CASE("segment_hits: through, over, short of, inside and past the corner of a box") {
  Box b{{0, 0, 0}, {1, 1, 1}};
  CHECK(segment_hits({-1, 0.5, 0.5}, {2, 0.5, 0.5}, b));     // straight through
  CHECK(!segment_hits({-1, 0.5, 1.5}, {2, 0.5, 1.5}, b));    // over the top
  CHECK(!segment_hits({-1, 0.5, 0.5}, {-0.1, 0.5, 0.5}, b)); // stops short
  CHECK(segment_hits({0.2, 0.2, 0.2}, {0.8, 0.8, 0.8}, b));  // all inside
  CHECK(!segment_hits({-1, 1.5, 0.5}, {2, 1.5, 0.5}, b));    // runs alongside
  // diagonals near the x = 0, y = 0 edge: the first misses it by 0.1, the second clips it
  CHECK(!segment_hits({-0.5, 0.4, 0.5}, {0.4, -0.5, 0.5}, b));
  CHECK(segment_hits({-0.5, 0.6, 0.5}, {0.6, -0.5, 0.5}, b));
}

// a random free cell center on the city grid, picked by the given generator
static Vec3 free_spot(const Grid& g, std::mt19937& rng) {
  while (true) {
    int i = rng() % g.nx, j = rng() % g.ny, k = rng() % g.nz;
    if (!g.blocked(i, j, k)) return g.center(i, j, k);
  }
}

// every box made bigger by m
static std::vector<Box> grown(const std::vector<Box>& boxes, double m) {
  std::vector<Box> out;
  for (const Box& b : boxes) out.push_back(grow(b, m));
  return out;
}

TEST_CASE("smooth: with nothing in the way the whole path becomes one straight segment") {
  Grid g = open_grid(10, 10, 10);
  auto raw = astar(g, g.center(0, 0, 0), g.center(9, 4, 2));
  auto path = smooth(raw, {});
  REQUIRE(path.size() == 2);
  CHECK(length(path) == doctest::Approx(norm(g.center(9, 4, 2) - g.center(0, 0, 0))));
}

TEST_CASE("smooth: on seeded cities the smoothed path keeps its ends, is no longer, and never cuts a building") {
  std::mt19937 rng(7);
  int planned = 0;
  for (uint32_t seed = 0; seed < 30; seed++) {
    auto boxes = city(seed);
    Grid g = city_grid(boxes);
    auto raw = astar(g, free_spot(g, rng), free_spot(g, rng));
    if (raw.empty()) continue;  // some random spots are boxed in by a building and the ceiling
    planned++;
    auto path = smooth(raw, grown(boxes, kMargin));
    CHECK(norm(path.front() - raw.front()) < 1e-12);
    CHECK(norm(path.back() - raw.back()) < 1e-12);
    CHECK(path.size() <= raw.size());
    CHECK(length(path) <= length(raw) + 1e-9);
    // the real safety check: the drone's center stays farther than its reach from every building
    CHECK(!path_blocked(path, grown(boxes, kReach)));
    for (size_t i = 1; i < path.size(); i++)
      for (const Box& b : grown(boxes, kReach)) CHECK(!segment_hits(path[i - 1], path[i], b));
  }
  CHECK(planned >= 20);
}

TEST_CASE("replan: a box dropped on the path is noticed, one off the path isn't, and the new path goes around") {
  auto boxes = city(3);
  Grid g = city_grid(boxes);
  Vec3 start = g.center(g.ix(0.25), g.iy(0.25), g.iz(5.25));
  Vec3 goal = g.center(g.ix(18.25), g.iy(18.25), g.iz(5.25));  // a street crossing
  auto path = smooth(astar(g, start, goal), grown(boxes, kMargin));
  REQUIRE(path.size() >= 2);
  CHECK(!path_blocked(path, grown(boxes, kReach)));

  // a 2 m cube right on the middle of the longest leg, which is long enough that the cube misses both ends
  size_t leg = 1;
  for (size_t i = 2; i < path.size(); i++)
    if (norm(path[i] - path[i - 1]) > norm(path[leg] - path[leg - 1])) leg = i;
  REQUIRE(norm(path[leg] - path[leg - 1]) > 6);
  Vec3 mid = (path[leg - 1] + path[leg]) * 0.5;
  Box dropped{mid - Vec3{1, 1, 1}, mid + Vec3{1, 1, 1}};
  CHECK(path_blocked(path, {grow(dropped, kReach)}));
  // the same cube moved to the far corner of the map, away from the path
  Box far{{-29, -29, 1}, {-27, -27, 3}};
  CHECK(!path_blocked(path, {grow(far, kReach)}));

  // mark it and plan again from the start: the new path stays clear of it
  mark(g, grow(dropped, kMargin));
  boxes.push_back(dropped);
  auto again = smooth(astar(g, start, goal), grown(boxes, kMargin));
  REQUIRE(!again.empty());
  CHECK(!path_blocked(again, grown(boxes, kReach)));
}

TEST_CASE("segment_hits: touching counts as a hit, lying on a face, a single point") {
  Box b{{0, 0, 0}, {1, 1, 1}};
  CHECK(segment_hits({-1, 0.5, 0.5}, {0, 0.5, 0.5}, b));     // ends right on a face
  CHECK(segment_hits({-1, 0, 0.5}, {2, 0, 0.5}, b));         // slides along the y = 0 face
  CHECK(!segment_hits({-1, -0.01, 0.5}, {2, -0.01, 0.5}, b)); // 1 cm outside that face
  CHECK(segment_hits({0.5, 0.5, 0.5}, {0.5, 0.5, 0.5}, b));  // a point inside
  CHECK(segment_hits({1, 0.5, 0.5}, {1, 0.5, 0.5}, b));      // a point on a face
  CHECK(!segment_hits({1.5, 0.5, 0.5}, {1.5, 0.5, 0.5}, b)); // a point outside
}

TEST_CASE("crash check: the drone's reach touching a box or the ground is a crash, a hair off is not") {
  std::vector<Box> world = {{{0, 0, 0}, {2, 2, 2}}};
  // off the side face
  CHECK(!crashed({2 + kReach + 0.01, 1, 1}, world));
  CHECK(crashed({2 + kReach - 0.01, 1, 1}, world));
  // off the top corner, diagonally: within reach on each axis but not in a straight line
  double d = 1 / std::sqrt(3.0);
  CHECK(!crashed(Vec3{2, 2, 2} + Vec3{d, d, d} * (kReach + 0.01), world));
  CHECK(crashed(Vec3{2, 2, 2} + Vec3{d, d, d} * (kReach - 0.01), world));
  // inside the box
  CHECK(crashed({1, 1, 1}, world));
  // the ground
  CHECK(!crashed({10, 10, kReach + 0.01}, world));
  CHECK(crashed({10, 10, kReach - 0.01}, world));
}
