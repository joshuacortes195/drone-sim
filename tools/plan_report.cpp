// prints the planning numbers for results/planning.md
#include "controller.h"
#include "estimator.h"
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
  double est_rms = 0, est_worst = 0, gps_rms = 0;  // only when flying on the estimate
};

// flies the path with the real physics and controller, the setpoint sliding along it at 5 m/s with 3 m/s^2 ramps,
// then 3 s more to settle. scores distance to the setpoint, the closest the drone's center gets to a building
// or the ground, and how far off the end it is at the finish. given sensor noise, it flies on what the sensors and
// estimator think instead of the true state, the noise drawn from this seed
static Flight fly(const std::vector<Vec3>& path, const std::vector<Box>& boxes, const Noise* noise, unsigned seed) {
  bool on_estimate = noise != nullptr;
  QuadParams P;
  Controller c;
  QuadState s;
  s.p = path[0];
  s.q = axis_angle({0, 0, 1}, follow(path, 0, 5, 3).yaw);
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  s.rotor = {w, w, w, w};

  // sensors and estimator, the gyro calibrated for 1 s on the pad first
  Noise n = noise ? *noise : Noise{};
  Sensors sn = make_sensors(n, seed);
  Estimator est = make_estimator(s, n);
  Vec3 gsum;
  for (int i = 0; i < 1000; i++) gsum = gsum + read_imu(sn, P, s, 0.001).gyro;
  est.gyro_bias = gsum * 0.001;
  double est_sum = 0, est_worst = 0, gps_sum = 0;
  int fixes = 0;

  int steps = int(std::lround((path_time(path, 5, 3) + 3) / 0.001));
  double sum = 0, worst = 0, clearance = 1e9;
  for (int i = 1; i <= steps; i++) {
    Setpoint sp = follow(path, i * 0.001, 5, 3);
    if (on_estimate) {
      // sensors feed the estimator: imu every step, gps and magnetometer at 10 Hz
      predict(est, read_imu(sn, P, s, 0.001), 0.001);
      if (i % 100 == 0) {
        Vec3 fix = read_gps(sn, s);
        gps(est, fix);
        mag(est, read_mag(sn, s));
        est_sum += dot(est.p - s.p, est.p - s.p);
        gps_sum += dot(fix - s.p, fix - s.p);
        fixes++;
      }
      est_worst = std::max(est_worst, norm(est.p - s.p));
    }
    s = step(P, s, control(c, P, on_estimate ? estimate(est) : s, sp, 0.001), 0.001);
    double e = norm(s.p - sp.pos);
    sum += e * e;
    worst = std::max(worst, e);
    clearance = std::min(clearance, s.p.z);
    for (const Box& b : boxes) clearance = std::min(clearance, box_dist(s.p, b));
  }
  Flight f{std::sqrt(sum / steps), worst, clearance, norm(s.p - path.back())};
  if (fixes) {
    f.est_rms = std::sqrt(est_sum / fixes);
    f.est_worst = est_worst;
    f.gps_rms = std::sqrt(gps_sum / fixes);
  }
  return f;
}

int main(int argc, char** argv) {
  const int seeds = 200;
  // "plan_report queries" lists each seed's start, goal and smoothed path, so the browser can redo the same plans
  bool list_queries = argc > 1 && std::string(argv[1]) == "queries";
  std::vector<double> grid_ms, astar_ms, smooth_ms, raw_ratio, smooth_ratio, points_raw, points_smooth, straight;
  std::vector<double> fly_rms, fly_worst, fly_clear, fly_end;
  // flying on the estimate, once with the neo-m8n gps and once with rtk
  struct Results {
    std::vector<double> worst, clear, end, est_rms, est_worst, gps_rms;
    int too_close = 0, inside = 0;
  } est[2];
  const Noise noises[2] = {Noise{}, rtk()};
  const char* names[2] = {"neo-m8n gps", "zed-f9p rtk gps"};
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
    Flight f = fly(path, boxes, nullptr, 0);
    fly_rms.push_back(f.rms);
    fly_worst.push_back(f.worst);
    fly_clear.push_back(f.clearance);
    fly_end.push_back(f.end_error);
    if (f.clearance < kReach) too_close++;

    // and again on the estimate, sensor noise seeded from the map seed
    for (int k = 0; k < 2; k++) {
      Flight e = fly(path, boxes, &noises[k], seed + 5000);
      Results& r = est[k];
      r.worst.push_back(e.worst);
      r.clear.push_back(e.clearance);
      r.end.push_back(e.end_error);
      r.est_rms.push_back(e.est_rms);
      r.est_worst.push_back(e.est_worst);
      r.gps_rms.push_back(e.gps_rms);
      if (e.clearance < kReach) r.too_close++;
      if (e.clearance < 0) r.inside++;
    }
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

  // the same flights on the estimate, once per gps
  for (int k = 0; k < 2; k++) {
    Results& r = est[k];
    std::printf("\nthe same flights on the estimate, %s (bmi088 imu, ist8310 magnetometer, gps at 10 Hz, 1 s gyro calibration):\n",
                names[k]);
    std::printf("flights within the drone's 0.356 m reach of a building or the ground: %d, center inside one: %d\n",
                r.too_close, r.inside);
    summary("worst to setpoint (m)", r.worst);
    summary("closest to building/ground (m)", r.clear);
    std::printf("%-30s %.3f m\n", "closest any flight got", *std::min_element(r.clear.begin(), r.clear.end()));
    summary("off the goal after 3 s (m)", r.end);
    summary("estimate rms error (m)", r.est_rms);
    summary("estimate worst error (m)", r.est_worst);
    summary("raw gps rms error (m)", r.gps_rms);
  }
}
