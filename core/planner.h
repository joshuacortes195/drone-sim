#pragma once

#include "map.h"

#include <vector>

// shortest cell path from start to goal through free cells, as cell centers, 26 neighbors.
// a blocked start starts from the closest free cell up to 4 cells (2 m) away.
// empty when the goal is blocked, the start is deeper in than that, or there's no way through
std::vector<Vec3> astar(const Grid& g, Vec3 start, Vec3 goal);

// drops every waypoint it can skip while the straight segments stay clear of the boxes
// (pass the boxes grown by kMargin). start and goal are kept
std::vector<Vec3> smooth(const std::vector<Vec3>& path, const std::vector<Box>& grown);

// true when any segment of the path touches any of the boxes, used to know when to replan
bool path_blocked(const std::vector<Vec3>& path, const std::vector<Box>& boxes);
