#pragma once

#include "quad.h"

// rotor speeds that make this total thrust (N) and body torque (N m). when the rotors can't do it all,
// thrust is kept, then roll and pitch, and yaw gives way first
Rotors mix(const QuadParams& P, double thrust, Vec3 torque);

// where the drone should be: a position, the path's own velocity and acceleration (zero to just hold) and a heading
struct Setpoint {
  Vec3 pos, vel, acc;
  double yaw = 0;
};

// tuning for all four loops, the outer ones run every outer_every physics steps
struct Gains {
  Vec3 pos = {1, 1, 1.5};          // position error (m) to velocity (m/s)
  Vec3 vel_p = {2.5, 2.5, 4};      // velocity error (m/s) to acceleration (m/s^2)
  Vec3 vel_i = {1, 1, 1};          // cancels steady pushes like rotor drag
  double vel_i_max = 2;            // most the velocity integral can push, m/s^2
  Vec3 att = {7, 7, 3};            // attitude error (rad) to spin rate (rad/s)
  Vec3 rate_p = {20, 20, 8};       // spin rate error (rad/s) to angular acceleration (rad/s^2)
  Vec3 rate_i = {10, 10, 2};
  double rate_i_max = 1;           // most the rate integral can push, rad/s^2, about twice the rotor drag torque at 5 m/s
  Vec3 max_rate = {4, 4, 0.8};     // rad/s, yaw kept slow so its weak torque can still stop it in time
  double max_speed_xy = 8, max_speed_z = 2;  // m/s, leaves room to catch up on a 5 m/s path
  double max_tilt = 0.6109;        // 35 degrees, well under the 51.6 where full throttle only just holds altitude
  double thrust_min = 0.2, thrust_max = 0.9;  // share of full thrust, leaves room to steer at both ends
  int outer_every = 20;            // position and velocity at 50 Hz on a 1 ms step
};

// what the controller remembers between steps
struct Controller {
  Gains g;
  Vec3 vel_int, rate_int;  // what the two integrals have built up
  Vec3 force_sp;           // world force the outer loop wants, held between its updates
  Quat att_sp;             // attitude the outer loop wants
  int tick = 0;
};

// runs the flight controller for one physics step of dt seconds, returns the rotor speed commands
Rotors control(Controller& c, const QuadParams& P, const QuadState& s, const Setpoint& sp, double dt);
