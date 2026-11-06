// flies the whole stack over many seeded cities for results/eval.md: plan, fly on the rtk estimate with the lidar on,
// replan when it sees something, in calm air and in wind. prints success rate, planning time, path length against a
// straight line, tracking error and estimation error
#include "controller.h"
#include "estimator.h"
#include "lidar.h"
#include "map.h"
#include "planner.h"
#include "quad.h"
#include "route.h"
#include "wind.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
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
  std::printf("  %-36s median %8.3f   p95 %8.3f   max %8.3f   (n = %zu)\n", name, v[v.size() / 2],
              v[std::min(v.size() - 1, v.size() * 95 / 100)], v.back(), v.size());
}

// total length of a path
static double length(const std::vector<Vec3>& path) {
  double L = 0;
  for (size_t i = 1; i < path.size(); i++) L += norm(path[i] - path[i - 1]);
  return L;
}

// the settings for the whole run, printed with the results
const double CRUISE = 5, ACCEL = 3, DT = 0.001;
// a flight gets this long in all, then it counts as out of time
const double TIME_LIMIT_S = 120;
// success means ending this close to the goal, with no crash, inside the time limit
const double ARRIVED_M = 1;
// the windy runs: a breeze from the west plus gusts, the middle of the wind_report grid
const double BREEZE = 4, GUSTS = 1.5;

// what happened on one flight
struct Flight {
  bool crashed = false, timed_out = false, stuck = false;
  double end_error = 0, track_rms = 0, est_rms = 0, seconds = 0;
  // the most the drone tipped from level on the whole flight, degrees
  double max_tilt = 0;
  // how far from the goal it was when a replan found no path, and how long before the path's end that was
  double stuck_dist = 0, stuck_early_s = 0;
  int replans = 0;
  std::vector<double> replan_ms;
};

// flies a planned path to goal through the city on the rtk estimate with the lidar on, in this wind
static Flight fly(std::vector<Vec3> path, Vec3 goal, const std::vector<Box>& boxes, double breeze, double gusts,
                  unsigned seed) {
  QuadParams P;
  Controller c;
  QuadState s;
  s.p = path[0];
  double yaw = follow(path, 0, CRUISE, ACCEL).yaw;
  s.q = axis_angle({0, 0, 1}, yaw);
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  s.rotor = {w, w, w, w};

  // what the drone knows: the city grid and the grown buildings
  Grid g = city_grid(boxes);
  std::vector<Box> grown;
  for (const Box& b : boxes) grown.push_back(grow(b, kMargin));

  // rtk sensors and the estimator, the gyro calibrated for 1 s first like the page, and the wind starting calm
  Noise n = rtk();
  Sensors sn = make_sensors(n, seed);
  Estimator est = make_estimator(s, n);
  Vec3 gsum;
  for (int i = 0; i < 1000; i++) gsum = gsum + read_imu(sn, P, s, DT).gyro;
  est.gyro_bias = gsum * DT;
  // the gust stream gets its own seed range, well away from every trip's sensor seed
  Wind wnd = make_wind({breeze, 0, 0}, gusts, seed + 10000);
  Lidar L;

  Flight f;
  double track_sum = 0, est_sum = 0;
  // when the current path started, and when to stop: its flight time plus 3 s to settle
  int start_i = 0, end_i = int(std::lround((path_time(path, CRUISE, ACCEL) + 3) / DT));
  const int limit_i = int(std::lround(TIME_LIMIT_S / DT));
  // steps actually flown, for the rms and the flight time
  int done = 0;
  for (int i = 1; i <= end_i; i++) {
    if (i > limit_i) {
      f.timed_out = true;
      break;
    }
    double t = (i - start_i) * DT;
    // the wind this step, felt by the imu and the physics alike
    Vec3 air = wind_step(wnd, DT);
    // imu every step, gps and magnetometer at 10 Hz
    predict(est, read_imu(sn, P, s, DT, air), DT);
    if (i % 100 == 0) {
      gps(est, read_gps(sn, s));
      mag(est, read_mag(sn, s));
    }
    QuadState e = estimate(est);

    // lidar at 10 Hz, half a period off the gps, and a replan when new cells cut the path (core/lidar.cpp's rule)
    if (i % 100 == 50) {
      size_t known = grown.size();
      bool added = add_hits(g, grown, scan(L, sn, s, e, boxes), boxes, e.p);
      auto t0 = std::chrono::steady_clock::now();
      if (added && replan(path, g, grown, known, e.p, goal)) {
        f.replan_ms.push_back(ms_since(t0));
        f.replans++;
        if (path.empty()) {
          // nowhere to go: hold where it is, and note where that happened
          f.stuck = true;
          f.stuck_dist = norm(e.p - goal);
          f.stuck_early_s = std::max(0.0, (end_i - i) * DT - 3);
          path = {e.p};
        }
        yaw = heading(e.q);
        start_i = i;
        end_i = i + int(std::lround((path_time(path, CRUISE, ACCEL) + 3) / DT));
        t = 0;
      }
    }

    Setpoint sp = follow(path, t, CRUISE, ACCEL, yaw);
    s = step(P, s, control(c, P, e, sp, DT), DT, air);
    done++;
    // how far the real drone is from where it was told to be, and from where it thinks it is
    track_sum += norm(s.p - sp.pos) * norm(s.p - sp.pos);
    est_sum += norm(e.p - s.p) * norm(e.p - s.p);
    // how far it's tipped from level right now
    f.max_tilt = std::max(f.max_tilt, std::acos(std::clamp(rotate(s.q, {0, 0, 1}).z, -1.0, 1.0)) * 180 / 3.14159265358979);
    // a crash ends the flight, like the page
    if (crashed(s.p, boxes)) {
      f.crashed = true;
      break;
    }
  }
  int steps = std::max(1, done);
  f.seconds = steps * DT;
  f.track_rms = std::sqrt(track_sum / steps);
  f.est_rms = std::sqrt(est_sum / steps);
  f.end_error = norm(s.p - goal);
  return f;
}

// everything recorded for one wind setting
struct Tally {
  int flights = 0, success = 0, crashed = 0, missed = 0, timed_out = 0, stuck = 0;
  std::vector<double> track, est, end, seconds, replans, replan_ms;
  std::vector<unsigned> failed_seeds;
  // seed, distance to the goal and seconds before the path's end, for each replan that found no path
  std::vector<std::array<double, 3>> stuck_at;
  // the crashes, kept out of the error stats so a crash doesn't read as a big tracking or goal error
  std::vector<double> crash_end;
  // which seed gave the worst tracking, the worst miss of the goal and the most tilt, so they can be flown again alone
  double worst_track = -1, worst_end = -1, worst_tilt = -1;
  unsigned worst_track_seed = 0, worst_end_seed = 0, worst_tilt_seed = 0;
  std::vector<double> tilt;
};

// adds one flight to the tally
static void count(Tally& t, const Flight& f, unsigned seed) {
  t.flights++;
  bool ok = !f.crashed && !f.timed_out && f.end_error < ARRIVED_M;
  if (ok) t.success++;
  else t.failed_seeds.push_back(seed);
  if (f.crashed) t.crashed++;
  else if (f.timed_out) t.timed_out++;
  else if (f.end_error >= ARRIVED_M) t.missed++;
  if (f.stuck) {
    t.stuck++;
    t.stuck_at.push_back({double(seed), f.stuck_dist, f.stuck_early_s});
  }
  if (f.crashed) {
    t.crash_end.push_back(f.end_error);
    return;
  }
  // keeps track of the worst flights
  if (f.track_rms > t.worst_track) t.worst_track = f.track_rms, t.worst_track_seed = seed;
  if (f.end_error > t.worst_end) t.worst_end = f.end_error, t.worst_end_seed = seed;
  if (f.max_tilt > t.worst_tilt) t.worst_tilt = f.max_tilt, t.worst_tilt_seed = seed;
  t.tilt.push_back(f.max_tilt);
  t.track.push_back(f.track_rms);
  t.est.push_back(f.est_rms);
  t.end.push_back(f.end_error);
  t.seconds.push_back(f.seconds);
  t.replans.push_back(f.replans);
  t.replan_ms.insert(t.replan_ms.end(), f.replan_ms.begin(), f.replan_ms.end());
}

// prints one wind setting's results
static void report(const char* name, const Tally& t) {
  std::printf("\n%s: %d flights, success %d (%.1f%%), crashed %d, out of time %d, ended over %.0f m from the goal %d, "
              "a replan found no path %d\n",
              name, t.flights, t.success, 100.0 * t.success / std::max(1, t.flights), t.crashed, t.timed_out,
              ARRIVED_M, t.missed, t.stuck);
  for (unsigned s : t.failed_seeds) std::printf("  failed on seed %u\n", s);
  for (double d : t.crash_end) std::printf("  a crash ended %.3f m from the goal\n", d);
  for (auto& s : t.stuck_at)
    std::printf("  no path on seed %.0f, %.2f m from the goal, %.2f s before the path's end\n", s[0], s[1], s[2]);
  std::printf("  the stats below leave out crashed flights. tracking error includes the 3 s hold at the goal\n");
  summary("tracking error, rms per flight (m)", t.track);
  summary("estimation error, rms per flight (m)", t.est);
  summary("off the goal at the end (m)", t.end);
  summary("flight time (s)", t.seconds);
  summary("replans per flight", t.replans);
  summary("replan: a* + smooth (ms)", t.replan_ms);
  summary("peak tilt per flight (deg)", t.tilt);
  if (!t.tilt.empty())
    std::printf("  worst flights: tracking on seed %u, off the goal on seed %u, tilt on seed %u\n", t.worst_track_seed,
                t.worst_end_seed, t.worst_tilt_seed);
}

int main(int argc, char** argv) {
  // how many seeded cities to fly, 500 unless told otherwise
  const int seeds = argc > 1 ? std::atoi(argv[1]) : 500;
  // the first seed, so one trip can be flown again on its own: batch_eval 1 962
  const int first = argc > 2 ? std::atoi(argv[2]) : 0;
  auto run_start = std::chrono::steady_clock::now();
  int no_path = 0;
  std::vector<double> plan_ms, ratio, straight;
  Tally calm, windy;

  for (uint32_t seed = uint32_t(first); seed < uint32_t(first + seeds); seed++) {
    // a seeded city and a random start and goal at least 20 m apart in free space. a different random stream from
    // plan_report and lidar_report, so these are new trips
    std::vector<Box> boxes = city(seed);
    Grid g = city_grid(boxes);
    std::mt19937 rng(seed + 20000);
    auto spot = [&] {
      while (true) {
        int i = rng() % g.nx, j = rng() % g.ny, k = rng() % g.nz;
        if (!g.blocked(i, j, k)) return g.center(i, j, k);
      }
    };
    Vec3 a = spot(), b = spot();
    while (norm(b - a) < 20) b = spot();

    // plans once and times it, a planning failure counts against the success rate in both winds
    std::vector<Box> grown;
    for (const Box& box : boxes) grown.push_back(grow(box, kMargin));
    auto t0 = std::chrono::steady_clock::now();
    auto path = smooth(astar(g, a, b), grown);
    plan_ms.push_back(ms_since(t0));
    if (path.empty()) {
      no_path++;
      calm.flights++;
      windy.flights++;
      calm.failed_seeds.push_back(seed);
      windy.failed_seeds.push_back(seed);
      continue;
    }
    ratio.push_back(length(path) / norm(b - a));
    straight.push_back(norm(b - a));

    // the same trip and sensor noise, once in calm air and once in the wind
    count(calm, fly(path, b, boxes, 0, 0, seed + 30000), seed);
    count(windy, fly(path, b, boxes, BREEZE, GUSTS, seed + 30000), seed);
  }

  std::printf("%d seeded cities (seeds %d to %d), one random trip each, start and goal in free space at least 20 m apart\n",
              seeds, first, first + seeds - 1);
  std::printf("rtk estimate, lidar on with replanning, %.0f m/s cruise, %.0f m/s^2 ramps, %.0f ms physics, "
              "success = within %.0f m of the goal, no crash, inside %.0f s\n",
              CRUISE, ACCEL, DT * 1000, ARRIVED_M, TIME_LIMIT_S);
  std::printf("windy = %.1f m/s breeze from the west plus %.2f m/s gusts\n", BREEZE, GUSTS);
  std::printf("\nplanning: no path found %d of %d\n", no_path, seeds);
  summary("plan: a* + smooth (ms)", plan_ms);
  summary("path length / straight line", ratio);
  summary("straight line distance (m)", straight);
  report("calm", calm);
  report("windy", windy);
  std::printf("\nwhole run took %.1f s\n", ms_since(run_start) / 1000);
}
