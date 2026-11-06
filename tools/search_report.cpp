// flies the page's lawnmower search the way the page does (its own bindings, built natively) and measures how much of
// the search area the drone really passed over, for results/search.md
#include "map.h"
#include "route.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

// the page's side of the sim (wasm/bindings.cpp)
extern "C" {
void reset(double x, double y, double z, double yaw);
int new_city(unsigned seed);
void advance(int steps);
int fly_to(double x, double y, double z);
int path_done();
int path_len();
double* path_ptr();
int replan_count();
int is_crashed();
void set_wind(double breeze, double gusts);
double* state_now();
double* est_now();
}

// the page's search: the whole flying area 6 m up, rows at most 12 m apart, then home 2 m over the pad
static const double EDGE = 29, Z = 6, SPACING = 12, ARRIVED_M = 1;
static const Vec3 HOME{0, 0, 2};
// a ground cell counts as searched when the drone passed within half the row spacing of it, sideways
static const double HALF = SPACING / 2;
// coverage grid cells, and how often the track is sampled
static const double CELL = 0.25;
static const int SAMPLE_STEPS = 10;
// gives up on a leg after this long
static const int LEG_STEPS = 60000;

// counts the cells of the search square within HALF of any track point, over the whole square and over open ground
// only (cells not under a building, where something on the ground could be seen at all)
static void coverage(const std::vector<Vec3>& track, const std::vector<Box>& buildings, double& all, double& open) {
  const int n = int(std::lround(2 * EDGE / CELL));
  std::vector<char> hit(size_t(n) * n, 0);
  // marks every cell whose center is within HALF of this track point
  const int r = int(std::ceil(HALF / CELL));
  for (const Vec3& p : track) {
    const int ci = int(std::floor((p.x + EDGE) / CELL)), cj = int(std::floor((p.y + EDGE) / CELL));
    for (int i = std::max(0, ci - r); i <= std::min(n - 1, ci + r); i++)
      for (int j = std::max(0, cj - r); j <= std::min(n - 1, cj + r); j++) {
        const double x = -EDGE + (i + 0.5) * CELL, y = -EDGE + (j + 0.5) * CELL;
        if ((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y) <= HALF * HALF) hit[size_t(i) * n + j] = 1;
      }
  }
  // adds up the covered share, and again skipping cells under a building
  int got = 0, open_n = 0, open_got = 0;
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++) {
      const double x = -EDGE + (i + 0.5) * CELL, y = -EDGE + (j + 0.5) * CELL;
      const bool c = hit[size_t(i) * n + j];
      got += c;
      bool under = false;
      for (const Box& b : buildings) under |= x > b.lo.x && x < b.hi.x && y > b.lo.y && y < b.hi.y;
      if (!under) open_n++, open_got += c;
    }
  all = 100.0 * got / (double(n) * n);
  open = 100.0 * open_got / open_n;
}

// total length of a list of points walked in order
static double length(const std::vector<Vec3>& pts) {
  double L = 0;
  for (size_t i = 1; i < pts.size(); i++) L += norm(pts[i] - pts[i - 1]);
  return L;
}

// how far a point is from the planned rows sideways: the closest of the row segments, ignoring height
static double off_rows(Vec3 p, const std::vector<Vec3>& rows) {
  double best = 1e9;
  for (size_t i = 1; i < rows.size(); i++) {
    const Vec3 a{rows[i - 1].x, rows[i - 1].y, 0}, b{rows[i].x, rows[i].y, 0}, q{p.x, p.y, 0};
    const double t = std::clamp(dot(q - a, b - a) / dot(b - a, b - a), 0.0, 1.0);
    best = std::min(best, norm(q - (a + (b - a) * t)));
  }
  return best;
}

// what one search flight came to
struct Result {
  double all = 0, open = 0, secs = 0, flown = 0, off = 0;
  int skipped = 0, replans = 0;
  bool crashed = false, stuck = false, done = false;
};

// flies one search over the seeded city from the page's start (hovering 1.5 m over home) and records the real track
// from reaching the first row to reaching the last one
static Result search(unsigned seed, double breeze, double gusts) {
  Result r;
  new_city(seed);
  // the panel's wind sliders, reset keeps them and starts the gusts calm from the same seed like the page
  set_wind(breeze, gusts);
  reset(0, 0, 1.5, 0);
  std::vector<Vec3> rows = lawnmower(-EDGE, -EDGE, EDGE, EDGE, Z, SPACING);
  std::vector<Vec3> goals = rows;
  goals.push_back(HOME);
  std::vector<Vec3> track;
  bool sweeping = false;
  int sweep_steps = 0;

  for (size_t g = 0; g < goals.size() && !r.crashed && !r.stuck; g++) {
    const Vec3 goal = goals[g];
    // a goal it can't plan to gets skipped, like the page does
    if (fly_to(goal.x, goal.y, goal.z) == 0) {
      r.skipped++;
      continue;
    }
    int replans = replan_count();
    bool arrived = false;
    for (int k = 0; k < LEG_STEPS && !arrived; k++) {
      advance(1);
      if (is_crashed()) {
        r.crashed = true;
        break;
      }
      const double* s = state_now();
      // keeps the real track while it's sweeping the rows
      if (sweeping) {
        sweep_steps++;
        if (sweep_steps % SAMPLE_STEPS == 0) track.push_back({s[0], s[1], s[2]});
      }
      // a failed replan leaves a one point hold. the page holds there for good unless it's within 1 m of the goal
      if (replan_count() != replans) {
        replans = replan_count();
        const double* e = est_now();
        const Vec3 d{e[0] - goal.x, e[1] - goal.y, e[2] - goal.z};
        if (path_len() <= 1 && dot(d, d) >= ARRIVED_M * ARRIVED_M) {
          r.stuck = true;
          break;
        }
      }
      // done with this goal once the path is flown and it's within 1 m of the path's end, same as the page
      if (path_done()) {
        const double* end = path_ptr() + 3 * (path_len() - 1);
        const Vec3 d{s[0] - end[0], s[1] - end[1], s[2] - end[2]};
        arrived = dot(d, d) < ARRIVED_M * ARRIVED_M;
      }
    }
    if (!arrived) {
      r.stuck = r.stuck || !r.crashed;
      break;
    }
    // the sweep starts at the first row's start and ends at the last row's end
    if (g == 0) sweeping = true;
    if (g + 1 == rows.size()) sweeping = false;
  }
  r.done = !r.crashed && !r.stuck;
  r.replans = replan_count();
  r.secs = sweep_steps * 0.001;
  r.flown = length(track);
  // the farthest the real track got from the planned rows
  for (const Vec3& p : track) r.off = std::max(r.off, off_rows(p, rows));
  coverage(track, city(seed), r.all, r.open);
  return r;
}

int main(int argc, char** argv) {
  const int seeds = argc > 1 ? std::atoi(argv[1]) : 1;
  // optional breeze from the west and gust strength, m/s
  const double breeze = argc > 2 ? std::atof(argv[2]) : 0, gusts = argc > 3 ? std::atof(argv[3]) : 0;
  std::printf("breeze %.1f m/s from the west, gusts %.2f m/s\n", breeze, gusts);

  // the cross check: the planned rows themselves, walked every 0.1 m, should cover the whole square
  std::vector<Vec3> rows = lawnmower(-EDGE, -EDGE, EDGE, EDGE, Z, SPACING);
  std::vector<Vec3> ideal;
  for (size_t i = 1; i < rows.size(); i++) {
    const Vec3 a = rows[i - 1], b = rows[i];
    const int n = int(std::ceil(norm(b - a) / 0.1));
    for (int k = 0; k <= n; k++) ideal.push_back(a + (b - a) * (double(k) / n));
  }
  double ia = 0, io = 0;
  coverage(ideal, city(1), ia, io);
  std::printf("planned rows: %zu points, %.1f m long, covers %.2f%% of the square (%.2f%% of open ground, seed 1)\n",
              rows.size(), length(rows), ia, io);

  // one search per seed, seed 1 is the city the page flies
  std::vector<double> all, open, off;
  int ok = 0;
  std::printf("seed  done  crash  stuck  skipped  replans  sweep s  flown m  max off rows m  covered %%  open ground %%\n");
  for (int sd = 1; sd <= seeds; sd++) {
    Result r = search(unsigned(sd), breeze, gusts);
    std::printf("%4d  %4d  %5d  %5d  %7d  %7d  %7.1f  %7.1f  %14.3f  %9.2f  %13.2f\n", sd, r.done, r.crashed,
                r.stuck, r.skipped, r.replans, r.secs, r.flown, r.off, r.all, r.open);
    if (r.done) ok++, all.push_back(r.all), open.push_back(r.open), off.push_back(r.off);
  }
  // spread over the searches that finished
  if (seeds > 1 && !all.empty()) {
    std::sort(all.begin(), all.end());
    std::sort(open.begin(), open.end());
    std::sort(off.begin(), off.end());
    std::printf("finished %d of %d. covered: min %.2f median %.2f max %.2f. open ground: min %.2f median %.2f max %.2f\n",
                ok, seeds, all.front(), all[all.size() / 2], all.back(), open.front(), open[open.size() / 2],
                open.back());
    std::printf("max off the rows: median %.3f p95 %.3f worst %.3f m\n", off[off.size() / 2],
                off[std::min(off.size() - 1, off.size() * 95 / 100)], off.back());
  }
}
