// prints the lidar and replanning numbers for results/lidar.md
#include "controller.h"
#include "estimator.h"
#include "lidar.h"
#include "map.h"
#include "planner.h"
#include "quad.h"
#include "route.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <random>
#include <vector>

// milliseconds since t
static double ms_since(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

// prints the median, 95th percentile and worst of a list
static void summary(const char* name, std::vector<double> v) {
  if (v.empty()) return;
  std::sort(v.begin(), v.end());
  std::printf("%-34s median %8.3f   p95 %8.3f   max %8.3f   (n = %zu)\n", name, v[v.size() / 2],
              v[std::min(v.size() - 1, v.size() * 95 / 100)], v.back(), v.size());
}

// distance from a point to the outside of a box, 0 when inside
static double box_dist(Vec3 p, const Box& b) {
  double dx = std::max({b.lo.x - p.x, 0.0, p.x - b.hi.x});
  double dy = std::max({b.lo.y - p.y, 0.0, p.y - b.hi.y});
  double dz = std::max({b.lo.z - p.z, 0.0, p.z - b.hi.z});
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// what happened on one flight
struct Flight {
  double clearance = 1e9, to_dropped = 1e9, end_error = 0;
  double seen_at = -1;  // how far the drone was from the dropped box when it first replanned around it
  int replans = 0;
  bool stuck = false;   // a replan found no way through
  std::vector<double> scan_ms, replan_ms;
};

// flies the path on the rtk estimate through the real world (the city plus the dropped box), knowing only the city.
// with the lidar on it scans at 10 Hz, and replans from the estimate whenever new cells cut the path it's on
static Flight fly(std::vector<Vec3> path, Vec3 goal, const std::vector<Box>& city_boxes, const Box& dropped,
                  bool lidar, unsigned seed) {
  QuadParams P;
  Controller c;
  QuadState s;
  s.p = path[0];
  double yaw = follow(path, 0, 5, 3).yaw;
  s.q = axis_angle({0, 0, 1}, yaw);
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  s.rotor = {w, w, w, w};

  // what the drone knows: the city grid and the grown buildings
  Grid g = city_grid(city_boxes);
  std::vector<Box> grown;
  for (const Box& b : city_boxes) grown.push_back(grow(b, kMargin));
  std::vector<Box> world = city_boxes;
  world.push_back(dropped);

  // rtk sensors and the estimator, the gyro calibrated for 1 s on the pad first
  Noise n = rtk();
  Sensors sn = make_sensors(n, seed);
  Estimator est = make_estimator(s, n);
  Vec3 gsum;
  for (int i = 0; i < 1000; i++) gsum = gsum + read_imu(sn, P, s, 0.001).gyro;
  est.gyro_bias = gsum * 0.001;
  Lidar L;

  Flight f;
  // when the current path started, and when to stop: its flight time plus 3 s to settle, 120 s at most
  int start_i = 0, end_i = int(std::lround((path_time(path, 5, 3) + 3) / 0.001));
  for (int i = 1; i <= end_i && i <= 120000; i++) {
    double t = (i - start_i) * 0.001;
    // imu every step, gps and magnetometer at 10 Hz
    predict(est, read_imu(sn, P, s, 0.001), 0.001);
    if (i % 100 == 0) {
      gps(est, read_gps(sn, s));
      mag(est, read_mag(sn, s));
    }

    // lidar at 10 Hz, half a period off the gps so they don't land on the same step
    if (lidar && i % 100 == 50) {
      auto t0 = std::chrono::steady_clock::now();
      size_t known = grown.size();
      QuadState e = estimate(est);
      bool added = add_hits(g, grown, scan(L, sn, s, e, world), city_boxes, e.p);
      f.scan_ms.push_back(ms_since(t0));
      // only replan when the new cells cut the path it's on
      if (added && path_blocked(path, std::vector<Box>(grown.begin() + long(known), grown.end()))) {
        t0 = std::chrono::steady_clock::now();
        auto next = smooth(astar(g, e.p, goal), grown);
        f.replan_ms.push_back(ms_since(t0));
        if (f.seen_at < 0) f.seen_at = box_dist(s.p, dropped);
        f.replans++;
        if (next.empty()) {
          // nowhere to go: hold where it is
          f.stuck = true;
          next = {e.p};
        }
        path = next;
        yaw = heading(e.q);
        start_i = i;
        end_i = i + int(std::lround((path_time(path, 5, 3) + 3) / 0.001));
        t = 0;
      }
    }

    Setpoint sp = follow(path, t, 5, 3, yaw);
    s = step(P, s, control(c, P, estimate(est), sp, 0.001), 0.001);
    // how close the real drone gets to the ground, the buildings and the dropped box
    f.clearance = std::min(f.clearance, s.p.z);
    for (const Box& b : city_boxes) f.clearance = std::min(f.clearance, box_dist(s.p, b));
    f.to_dropped = std::min(f.to_dropped, box_dist(s.p, dropped));
  }
  f.clearance = std::min(f.clearance, f.to_dropped);
  f.end_error = norm(s.p - goal);
  return f;
}

int main() {
  const int seeds = 200;
  int no_path = 0, short_leg = 0, flown = 0;
  // lidar off: how often it flies into the box it can't see
  int off_inside = 0, off_close = 0;
  // lidar on
  int on_inside = 0, on_close = 0, on_goal = 0, on_stuck = 0, never_replanned = 0;
  std::vector<double> off_dropped, on_dropped, on_clear, seen_at, replans, scan_ms, replan_ms, on_end;

  for (uint32_t seed = 0; seed < seeds; seed++) {
    // the same city, start and goal per seed as plan_report
    std::vector<Box> boxes = city(seed);
    Grid g = city_grid(boxes);
    std::mt19937 rng(seed + 1000);
    auto spot = [&] {
      while (true) {
        int i = rng() % g.nx, j = rng() % g.ny, k = rng() % g.nz;
        if (!g.blocked(i, j, k)) return g.center(i, j, k);
      }
    };
    Vec3 a = spot(), b = spot();
    while (norm(b - a) < 20) b = spot();
    std::vector<Box> grown;
    for (const Box& box : boxes) grown.push_back(grow(box, kMargin));
    auto path = smooth(astar(g, a, b), grown);
    if (path.empty()) {
      no_path++;
      continue;
    }

    // a 2 m cube floating on the middle of the longest leg, if that leg is long enough to miss both ends
    size_t leg = 1;
    for (size_t i = 2; i < path.size(); i++)
      if (norm(path[i] - path[i - 1]) > norm(path[leg] - path[leg - 1])) leg = i;
    if (norm(path[leg] - path[leg - 1]) <= 6) {
      short_leg++;
      continue;
    }
    Vec3 mid = (path[leg - 1] + path[leg]) * 0.5;
    Box dropped{mid - Vec3{1, 1, 1}, mid + Vec3{1, 1, 1}};
    flown++;

    // once blind, once with the lidar, same sensor noise
    Flight off = fly(path, b, boxes, dropped, false, seed + 5000);
    off_dropped.push_back(off.to_dropped);
    if (off.to_dropped == 0) off_inside++;
    if (off.to_dropped < kReach) off_close++;

    Flight on = fly(path, b, boxes, dropped, true, seed + 5000);
    on_dropped.push_back(on.to_dropped);
    on_clear.push_back(on.clearance);
    on_end.push_back(on.end_error);
    if (on.clearance == 0) on_inside++;
    if (on.clearance < kReach) on_close++;
    if (on.end_error < 1) on_goal++;
    if (on.stuck) on_stuck++;
    if (on.seen_at >= 0) seen_at.push_back(on.seen_at);
    else never_replanned++;
    replans.push_back(on.replans);
    scan_ms.insert(scan_ms.end(), on.scan_ms.begin(), on.scan_ms.end());
    replan_ms.insert(replan_ms.end(), on.replan_ms.begin(), on.replan_ms.end());
  }

  std::printf("%d seeds, same starts and goals as plan_report. no path: %d, longest leg 6 m or shorter: %d, flown: %d\n",
              seeds, no_path, short_leg, flown);
  std::printf("a 2 m cube the drone doesn't know about, floating on the middle of the longest leg\n");
  std::printf("rtk estimate, 5 m/s cruise, 3 m/s^2 ramps, 1 ms physics, vlp-16 at 10 Hz (16 x 180 beams, 3 cm noise)\n\n");
  std::printf("lidar off: center inside the cube %d, within the %.3f m reach of it %d\n", off_inside, kReach, off_close);
  summary("lidar off, closest to the cube (m)", off_dropped);
  std::printf("\nlidar on: center inside anything %d, within reach of anything %d, ended within 1 m of the goal %d,\n",
              on_inside, on_close, on_goal);
  std::printf("  a replan found no path %d, never replanned %d\n", on_stuck, never_replanned);
  summary("closest to the cube (m)", on_dropped);
  summary("closest to anything (m)", on_clear);
  std::printf("%-34s %.3f m\n", "closest any flight got", *std::min_element(on_clear.begin(), on_clear.end()));
  summary("off the goal at the end (m)", on_end);
  summary("distance to cube at first replan (m)", seen_at);
  summary("replans per flight", replans);
  summary("scan + add_hits (ms)", scan_ms);
  summary("replan: a* + smooth (ms)", replan_ms);
}
