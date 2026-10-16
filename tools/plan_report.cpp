// prints the planning numbers for results/planning.md
#include "controller.h"
#include "map.h"
#include "planner.h"
#include "quad.h"
#include "route.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

// milliseconds since t
static double ms_since(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

// total length of a path
static double length(const std::vector<Vec3>& path) {
  double L = 0;
  for (size_t i = 1; i < path.size(); i++) L += norm(path[i] - path[i - 1]);
  return L;
}

// prints median, 95th percentile and worst of a list
static void summary(const char* name, std::vector<double> v) {
  std::sort(v.begin(), v.end());
  std::printf("%-30s median %8.3f   p95 %8.3f   max %8.3f   (n = %zu)\n", name, v[v.size() / 2],
              v[size_t(v.size() * 0.95)], v.back(), v.size());
}

// how far a point is from a box, 0 when inside it
static double box_dist(Vec3 p, const Box& b) {
  double dx = std::max({b.lo.x - p.x, 0.0, p.x - b.hi.x});
  double dy = std::max({b.lo.y - p.y, 0.0, p.y - b.hi.y});
  double dz = std::max({b.lo.z - p.z, 0.0, p.z - b.hi.z});
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// what happened when the drone actually flew a path
struct Flight {
  double rms, worst, clearance, end_error;
};

// flies the path with the real physics and controller, the setpoint sliding along it at 5 m/s with 3 m/s^2 ramps,
// then 3 s more to settle. scores distance to the setpoint, the closest the drone's center gets to a building
// or the ground, and how far off the end it is at the finish
static Flight fly(const std::vector<Vec3>& path, const std::vector<Box>& boxes) {
  QuadParams P;
  Controller c;
  QuadState s;
  s.p = path[0];
  s.q = axis_angle({0, 0, 1}, follow(path, 0, 5, 3).yaw);
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  s.rotor = {w, w, w, w};
  int steps = int(std::lround((path_time(path, 5, 3) + 3) / 0.001));
  double sum = 0, worst = 0, clearance = 1e9;
  for (int i = 1; i <= steps; i++) {
    Setpoint sp = follow(path, i * 0.001, 5, 3);
    s = step(P, s, control(c, P, s, sp, 0.001), 0.001);
    double e = norm(s.p - sp.pos);
    sum += e * e;
    worst = std::max(worst, e);
    clearance = std::min(clearance, s.p.z);
    for (const Box& b : boxes) clearance = std::min(clearance, box_dist(s.p, b));
  }
  return {std::sqrt(sum / steps), worst, clearance, norm(s.p - path.back())};
}

int main(int argc, char** argv) {
  const int seeds = 200;
  // "plan_report queries" lists each seed's start, goal and smoothed path, so the browser can redo the same plans
  bool list_queries = argc > 1 && std::string(argv[1]) == "queries";
  std::vector<double> grid_ms, astar_ms, smooth_ms, raw_ratio, smooth_ratio, points_raw, points_smooth, straight;
  std::vector<double> fly_rms, fly_worst, fly_clear, fly_end;
  int no_path = 0, blocked_after = 0, too_close = 0;

  for (uint32_t seed = 0; seed < seeds; seed++) {
    // one city per seed, and a start and goal picked from that seed, at least 20 m apart
    std::vector<Box> boxes = city(seed);
    auto t = std::chrono::steady_clock::now();
    Grid g = city_grid(boxes);
    grid_ms.push_back(ms_since(t));

    std::mt19937 rng(seed + 1000);
    auto spot = [&] {
      while (true) {
        int i = rng() % g.nx, j = rng() % g.ny, k = rng() % g.nz;
        if (!g.blocked(i, j, k)) return g.center(i, j, k);
      }
    };
    Vec3 a = spot(), b = spot();
    while (norm(b - a) < 20) b = spot();
    // plan, then smooth against the buildings grown by the full margin
    t = std::chrono::steady_clock::now();
    auto raw = astar(g, a, b);
    astar_ms.push_back(ms_since(t));
    if (raw.empty()) {
      no_path++;
      continue;
    }
    std::vector<Box> grown, reach;
    for (const Box& box : boxes) {
      grown.push_back(grow(box, kMargin));
      reach.push_back(grow(box, kReach));
    }
    t = std::chrono::steady_clock::now();
    auto path = smooth(raw, grown);
    smooth_ms.push_back(ms_since(t));
    // with the smoothed path's waypoint count and length, so the browser build can be checked against this one
    if (list_queries) {
      std::printf("%u %.17g %.17g %.17g %.17g %.17g %.17g %zu %.17g\n", seed, a.x, a.y, a.z, b.x, b.y, b.z,
                  path.size(), length(path));
      continue;
    }

    // how much longer than flying straight through the buildings, and a safety recheck
    double d = norm(b - a);
    straight.push_back(d);
    raw_ratio.push_back(length(raw) / d);
    smooth_ratio.push_back(length(path) / d);
    points_raw.push_back(double(raw.size()));
    points_smooth.push_back(double(path.size()));
    if (path_blocked(path, reach)) blocked_after++;

    // then fly it for real
    Flight f = fly(path, boxes);
    fly_rms.push_back(f.rms);
    fly_worst.push_back(f.worst);
    fly_clear.push_back(f.clearance);
    fly_end.push_back(f.end_error);
    if (f.clearance < kReach) too_close++;
  }

  if (list_queries) return 0;
  std::printf("%d seeds, one random start and goal each (>= 20 m apart), 60 x 60 x 20 m at %.1f m cells\n", seeds, kVoxel);
  std::printf("no path found: %d   smoothed paths touching a building grown by the drone's reach: %d\n\n", no_path,
              blocked_after);
  summary("build grid (ms)", grid_ms);
  summary("a* (ms)", astar_ms);
  summary("smooth (ms)", smooth_ms);
  summary("straight line distance (m)", straight);
  summary("a* length / straight", raw_ratio);
  summary("smoothed length / straight", smooth_ratio);
  summary("a* waypoints", points_raw);
  summary("smoothed waypoints", points_smooth);
  std::printf("\nflying each smoothed path (5 m/s cruise, 3 m/s^2 ramps, stops at waypoints, 1 ms physics):\n");
  std::printf("flights where the drone's center came within its 0.356 m reach of a building or the ground: %d\n",
              too_close);
  summary("rms to setpoint (m)", fly_rms);
  summary("worst to setpoint (m)", fly_worst);
  summary("closest to building/ground (m)", fly_clear);
  std::printf("%-30s %.3f m (the drone's reach is %.3f m)\n", "closest any flight got",
              *std::min_element(fly_clear.begin(), fly_clear.end()), kReach);
  summary("off the goal after 3 s (m)", fly_end);
}
