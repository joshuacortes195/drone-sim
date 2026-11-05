#pragma once

#include "controller.h"

#include <vector>

// seconds the reference needs for a leg of length L
double leg_time(double L, double cruise, double accel);

// how far along a leg of length L the reference is after t seconds, how fast it's going and speeding up.
// accel 0 means it jumps straight to cruise speed and turns corners instantly, otherwise it speeds up
// and slows down at accel so it stops at each waypoint (a triangle if the leg is too short to reach cruise)
void along(double L, double cruise, double accel, double t, double& dist, double& speed, double& acc);

// total seconds to fly a whole path, stopping at every waypoint
double path_time(const std::vector<Vec3>& path, double cruise, double accel);

// where the drone should be t seconds after starting down the path, with the path's velocity and
// acceleration fed forward and the nose pointed along the current leg. holds the last point once done.
// yaw is the heading to keep until the first leg that moves sideways
Setpoint follow(const std::vector<Vec3>& path, double t, double cruise, double accel, double yaw = 0);

// a lawnmower search over the box x0..x1, y0..y1 at height z: rows running east west, the first on y0 and the last on
// y1, spread evenly with no gap wider than spacing, each row flown the opposite way from the one before
std::vector<Vec3> lawnmower(double x0, double y0, double x1, double y1, double z, double spacing);
