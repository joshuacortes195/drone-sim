#include "quad.h"

#include <algorithm>

Wrench wrench(const QuadParams& P, const QuadState& s) {
  Wrench out;
  // how fast the drone moves through the air, seen from the body (wind gets subtracted here later)
  Vec3 air = rotate(conj(s.q), s.v);
  for (int i = 0; i < 4; i++) {
    // each rotor pushes straight up along body z, and tips the body from where it sits
    double thrust = P.k_f * s.rotor[i] * s.rotor[i];
    Vec3 lift{0, 0, thrust};
    out.force = out.force + lift;
    out.torque = out.torque + cross(P.rotor_pos[i], lift);
    // the air drags on the spinning rotor, which twists the body the other way
    out.torque.z += -P.spin[i] * P.moment_constant * thrust;
    // rotor drag: the rotor's own sideways air speed gets pushed back on, right at the rotor
    Vec3 va = air + cross(s.w, P.rotor_pos[i]);
    Vec3 side{va.x, va.y, 0};
    Vec3 drag = side * (-P.rotor_drag * std::abs(s.rotor[i]));
    out.force = out.force + drag;
    out.torque = out.torque + cross(P.rotor_pos[i], drag);
    // rolling moment, same as rotors: it ignores spin direction, so all four add up
    out.torque = out.torque + side * (-P.rolling_moment * std::abs(s.rotor[i]));
  }
  return out;
}

QuadState derivative(const QuadParams& P, const QuadState& s, const Rotors& cmd) {
  Wrench f = wrench(P, s);
  QuadState d;
  d.p = s.v;
  // newton: rotor force turned into the world, plus gravity
  d.v = rotate(s.q, f.force) * (1 / P.mass) + Vec3{0, 0, -9.81};
  // orientation changes with the body spin rate
  d.q = s.q * Quat{0, s.w.x, s.w.y, s.w.z} * 0.5;
  // euler: torque spins the body up, minus the wobble a spinning body makes on its own (w x Iw)
  Vec3 net = f.torque - cross(s.w, mul(P.inertia, s.w));
  d.w = Vec3{net.x / P.inertia.x, net.y / P.inertia.y, net.z / P.inertia.z};
  // motors can't jump to a new speed, each one chases its command, faster going up than down
  for (int i = 0; i < 4; i++) {
    double target = std::clamp(cmd[i], 0.0, P.max_rotor_speed);
    double tau = target > s.rotor[i] ? P.tau_up : P.tau_down;
    d.rotor[i] = (target - s.rotor[i]) / tau;
  }
  return d;
}

// state + rate * h, used by rk4 to make the in between states
static QuadState add(const QuadState& s, const QuadState& d, double h) {
  QuadState r;
  r.p = s.p + d.p * h;
  r.v = s.v + d.v * h;
  r.q = s.q + d.q * h;
  r.w = s.w + d.w * h;
  for (int i = 0; i < 4; i++) r.rotor[i] = s.rotor[i] + d.rotor[i] * h;
  return r;
}

QuadState step(const QuadParams& P, const QuadState& s, const Rotors& cmd, double dt) {
  // rk4: sample the rates at the start, twice in the middle and at the end, then blend
  QuadState k1 = derivative(P, s, cmd);
  QuadState k2 = derivative(P, add(s, k1, dt / 2), cmd);
  QuadState k3 = derivative(P, add(s, k2, dt / 2), cmd);
  QuadState k4 = derivative(P, add(s, k3, dt), cmd);
  QuadState r = add(s, k1, dt / 6);
  r = add(r, k2, dt / 3);
  r = add(r, k3, dt / 3);
  r = add(r, k4, dt / 6);
  // keeps the quaternion a real rotation after rounding error
  r.q = normalized(r.q);
  return r;
}
