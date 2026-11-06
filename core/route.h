#pragma once

#include "controller.h"

#include <vector>

// seconds the reference needs for a leg of length L
double leg_time(double L, double cruise, double accel);

// how far along a leg of length L the reference is after t seconds, how fast it's going and speeding up.
// accel 0 means it jumps straight to cruise speed and turns corners instantly, otherwise it speeds up
// and slows down at accel so it stops at each waypoint (a triangle if the leg is too short to reach cruise)
void along(double L, double cruise, double accel, double t, double& dist, double& speed, double& acc);

// how hard a path that starts at speed brakes first, m/s^2. well under the 6.9 the 35 degree tilt limit can do, so
// there's room left to steer
constexpr double kBrake = 5;

// total seconds to fly a whole path, stopping at every waypoint. v0 is how fast the drone is going when it starts,
// and then the first leg is a straight braking leg from it that ends at rest on the second point
double path_time(const std::vector<Vec3>& path, double cruise, double accel, Vec3 v0 = {});

// where the drone should be t seconds after starting down the path, with the path's velocity and
// acceleration fed forward and the nose pointed along the current leg. holds the last point once done.
// yaw is the heading to keep until the first leg that moves sideways. with a v0 the first leg is a straight braking
// leg: it starts at the drone's speed and slows at kBrake to rest on the second point, so a path planned while moving
// starts at the drone (replan() lays that leg out)
Setpoint follow(const std::vector<Vec3>& path, double t, double cruise, double accel, double yaw = 0, Vec3 v0 = {});

// a lawnmower search over the box x0..x1, y0..y1 at height z: rows running east west, the first on y0 and the last on
// y1, spread evenly with no gap wider than spacing, each row flown the opposite way from the one before
std::vector<Vec3> lawnmower(double x0, double y0, double x1, double y1, double z, double spacing);

// landing: where the drone's center sits once its skids touch the ground (they hang 0.115 m under it, see the model in
// web/src/scene.ts), and how fast the last leg comes down onto the pad, ramping in and out at the same rate
constexpr double kSkid = 0.115, kLandSpeed = 0.5;
