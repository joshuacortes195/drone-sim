#include "planner.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

std::vector<Vec3> astar(const Grid& g, Vec3 start, Vec3 goal) {
  int si = g.ix(start.x), sj = g.iy(start.y), sk = g.iz(start.z);
  int gi = g.ix(goal.x), gj = g.iy(goal.y), gk = g.iz(goal.z);
  if (g.blocked(si, sj, sk) || g.blocked(gi, gj, gk)) return {};

  // cells are numbered flat, same order as the occupancy list
  auto id = [&](int i, int j, int k) { return (k * g.ny + j) * g.nx + i; };
  Vec3 goal_c = g.center(gi, gj, gk);
  // straight line distance left, never more than the real distance so the path stays shortest
  auto h = [&](int i, int j, int k) { return norm(g.center(i, j, k) - goal_c); };

  // best known distance to each cell, and the cell we came from to get there
  std::vector<double> cost(g.occ.size(), std::numeric_limits<double>::infinity());
  std::vector<int> from(g.occ.size(), -1);
  std::vector<uint8_t> done(g.occ.size(), 0);

  // open cells, cheapest distance so far plus distance left comes out first
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  int s = id(si, sj, sk), t = id(gi, gj, gk);
  cost[s] = 0;
  open.push({h(si, sj, sk), s});

  while (!open.empty()) {
    int c = open.top().second;
    open.pop();
    if (done[c]) continue;  // an older, worse entry for a cell we already settled
    done[c] = 1;
    if (c == t) break;
    int ci = c % g.nx, cj = (c / g.nx) % g.ny, ck = c / (g.nx * g.ny);

    // try all 26 touching cells
    for (int dk = -1; dk <= 1; dk++)
      for (int dj = -1; dj <= 1; dj++)
        for (int di = -1; di <= 1; di++) {
          if (!di && !dj && !dk) continue;
          int ni = ci + di, nj = cj + dj, nk = ck + dk;
          // no cutting corners: every cell the move brushes past has to be free too
          bool clear = true;
          for (int a : {0, di})
            for (int b : {0, dj})
              for (int e : {0, dk}) clear = clear && !g.blocked(ci + a, cj + b, ck + e);
          if (!clear) continue;

          // a step is 0.5, 0.71 or 0.87 m depending on how many axes it moves along
          int n = id(ni, nj, nk);
          double step = g.res * std::sqrt(double(di * di + dj * dj + dk * dk));
          if (cost[c] + step < cost[n]) {
            cost[n] = cost[c] + step;
            from[n] = c;
            open.push({cost[n] + h(ni, nj, nk), n});
          }
        }
  }
  if (!done[t]) return {};

  // walk back from the goal to the start, then flip it around
  std::vector<Vec3> path;
  for (int c = t; c != -1; c = from[c]) path.push_back(g.center(c % g.nx, (c / g.nx) % g.ny, c / (g.nx * g.ny)));
  std::reverse(path.begin(), path.end());
  return path;
}

// true when the segment touches none of the boxes
static bool clear(Vec3 a, Vec3 b, const std::vector<Box>& boxes) {
  for (const Box& box : boxes)
    if (segment_hits(a, b, box)) return false;
  return true;
}

std::vector<Vec3> smooth(const std::vector<Vec3>& path, const std::vector<Box>& grown) {
  if (path.size() < 3) return path;
  // from each kept point, jump to the farthest point down the path it can see in a straight line.
  // the very next point is always allowed since the cell path itself is already safe
  // (with buildings bigger than a cell that fallback never actually gets used)
  std::vector<Vec3> out = {path[0]};
  size_t i = 0;
  while (i < path.size() - 1) {
    size_t j = path.size() - 1;
    while (j > i + 1 && !clear(path[i], path[j], grown)) j--;
    out.push_back(path[j]);
    i = j;
  }
  return out;
}

bool path_blocked(const std::vector<Vec3>& path, const std::vector<Box>& boxes) {
  for (size_t i = 1; i < path.size(); i++)
    if (!clear(path[i - 1], path[i], boxes)) return true;
  return false;
}
