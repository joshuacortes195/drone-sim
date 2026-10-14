#pragma once

#include "vecmath.h"

#include <array>

// one number per rotor, in rotors' iris order: 0 front right, 1 back left, 2 front left, 3 back right
using Rotors = std::array<double, 4>;

// the iris airframe from rotors (eth zurich), values from iris.xacro, see docs/plan.md
struct QuadParams {
  double mass = 1.52;                                   // 1.5 kg body plus four 0.005 kg rotors
  Vec3 inertia = {0.0347563, 0.0458929, 0.0977};        // body only, diagonal
  double k_f = 8.54858e-06;                             // thrust = k_f w^2
  double moment_constant = 0.016;                       // yaw twist = 0.016 m * thrust
  double max_rotor_speed = 838;                         // rad/s
  double tau_up = 0.0125, tau_down = 0.025;             // motor lag, s
  double rotor_drag = 8.06428e-05;                      // sideways drag per rotor
  double rolling_moment = 1e-06;
  // where each rotor sits in the body frame, all 2.3 cm above the center
  std::array<Vec3, 4> rotor_pos = {Vec3{0.13, -0.22, 0.023}, Vec3{-0.13, 0.20, 0.023},
                                   Vec3{0.13, 0.22, 0.023}, Vec3{-0.13, -0.20, 0.023}};
  // +1 counterclockwise, -1 clockwise, seen from above
  Rotors spin = {1, 1, -1, -1};
};

// everything that changes as the drone flies
struct QuadState {
  Vec3 p, v;   // world position and velocity
  Quat q;      // body to world rotation
  Vec3 w;      // body spin rate, rad/s
  Rotors rotor{};  // actual rotor speeds, rad/s
};

// total force and torque from the rotors, in the body frame (no gravity)
struct Wrench {
  Vec3 force, torque;
};

Wrench wrench(const QuadParams& P, const QuadState& s);

// how fast each part of the state is changing right now, packed in the same struct
QuadState derivative(const QuadParams& P, const QuadState& s, const Rotors& cmd);

// moves the drone forward dt seconds with rk4, cmd is the rotor speed each motor is asked for
QuadState step(const QuadParams& P, const QuadState& s, const Rotors& cmd, double dt);
