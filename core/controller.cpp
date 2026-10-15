#include "controller.h"

#include <algorithm>
#include <cmath>
#include <utility>

// thrust, roll, pitch, yaw packed together, or one thrust per rotor
using Vec4 = std::array<double, 4>;

// finds the four rotor thrusts that make b = (total thrust, roll, pitch, yaw torque),
// by solving the same rotor table wrench() uses, so the uneven arms come out exact
static Vec4 solve(const QuadParams& P, Vec4 b) {
  // each column is what one newton on that rotor does to thrust, roll, pitch and yaw
  double A[4][5];
  for (int i = 0; i < 4; i++) {
    A[0][i] = 1;
    A[1][i] = P.rotor_pos[i].y;
    A[2][i] = -P.rotor_pos[i].x;
    A[3][i] = -P.spin[i] * P.moment_constant;
  }
  for (int r = 0; r < 4; r++) A[r][4] = b[r];
  // gauss jordan with the biggest pivot each column, keeps rounding error small
  for (int c = 0; c < 4; c++) {
    int best = c;
    for (int r = c + 1; r < 4; r++)
      if (std::abs(A[r][c]) > std::abs(A[best][c])) best = r;
    std::swap(A[c], A[best]);
    for (int r = 0; r < 4; r++) {
      if (r == c) continue;
      double k = A[r][c] / A[c][c];
      for (int k2 = c; k2 < 5; k2++) A[r][k2] -= k * A[c][k2];
    }
  }
  return {A[0][4] / A[0][0], A[1][4] / A[1][1], A[2][4] / A[2][2], A[3][4] / A[3][3]};
}

// biggest share (0 to 1) of the change d that fits on top of thrusts t without any rotor
// going below zero or past full thrust
static double fit(const Vec4& t, const Vec4& d, double t_max) {
  double a = 1;
  for (int i = 0; i < 4; i++) {
    if (t[i] + d[i] > t_max) a = std::min(a, (t_max - t[i]) / d[i]);
    if (t[i] + d[i] < 0) a = std::min(a, -t[i] / d[i]);
  }
  return std::max(a, 0.0);
}

Rotors mix(const QuadParams& P, double thrust, Vec3 torque) {
  double t_max = P.k_f * P.max_rotor_speed * P.max_rotor_speed;
  // total thrust first, it's what keeps the drone in the air
  Vec4 t = solve(P, {std::clamp(thrust, 0.0, 4 * t_max), 0, 0, 0});
  // then as much roll and pitch as fits on top
  Vec4 rp = solve(P, {0, torque.x, torque.y, 0});
  double a = fit(t, rp, t_max);
  for (int i = 0; i < 4; i++) t[i] += a * rp[i];
  // yaw last, it's the weakest axis and the least dangerous to give up
  Vec4 yaw = solve(P, {0, 0, 0, torque.z});
  double b = fit(t, yaw, t_max);
  // thrust back to rotor speed, the max() keeps a rounding speck below zero out of the sqrt
  Rotors w;
  for (int i = 0; i < 4; i++) w[i] = std::sqrt(std::max(t[i] + b * yaw[i], 0.0) / P.k_f);
  return w;
}

// clamps each part of v to +-lim
static Vec3 clamp3(Vec3 v, Vec3 lim) {
  return {std::clamp(v.x, -lim.x, lim.x), std::clamp(v.y, -lim.y, lim.y), std::clamp(v.z, -lim.z, lim.z)};
}

// position and velocity loops: works out the force the drone should make and the attitude that points it there
static void outer_loop(Controller& c, const QuadParams& P, const QuadState& s, const Setpoint& sp, double dt) {
  const Gains& g = c.g;
  // position error becomes a velocity to fly at, on top of the path's own velocity, capped
  Vec3 v_sp = sp.vel + mul(g.pos, sp.pos - s.p);
  double h = std::hypot(v_sp.x, v_sp.y);
  if (h > g.max_speed_xy) v_sp = {v_sp.x * g.max_speed_xy / h, v_sp.y * g.max_speed_xy / h, v_sp.z};
  v_sp.z = std::clamp(v_sp.z, -g.max_speed_z, g.max_speed_z);
  // velocity error becomes an acceleration on top of the path's own, the integral is capped so it can't wind up
  Vec3 e = v_sp - s.v;
  double m = g.vel_i_max;
  c.vel_int = clamp3(c.vel_int + mul(g.vel_i, e) * dt, {m, m, m});
  Vec3 a = sp.acc + mul(g.vel_p, e) + c.vel_int;
  // force that makes that acceleration while holding up the weight
  Vec3 f = (a + Vec3{0, 0, 9.81}) * P.mass;
  // up and down first, then sideways gets whatever the tilt limit and the thrust left over allow
  double full = 4 * P.k_f * P.max_rotor_speed * P.max_rotor_speed;
  double top = g.thrust_max * full;
  f.z = std::clamp(f.z, g.thrust_min * full, top);
  double side = std::hypot(f.x, f.y);
  double side_max = std::min(f.z * std::tan(g.max_tilt), std::sqrt(top * top - f.z * f.z));
  if (side > side_max) f = {f.x * side_max / side, f.y * side_max / side, f.z};
  c.force_sp = f;
  // attitude: tip body z onto the force (shortest arc from world z), after turning to the heading
  Vec3 z = f * (1 / norm(f));
  c.att_sp = normalized(Quat{1 + z.z, -z.y, z.x, 0}) * axis_angle({0, 0, 1}, sp.yaw);
}

Rotors control(Controller& c, const QuadParams& P, const QuadState& s, const Setpoint& sp, double dt) {
  const Gains& g = c.g;
  // the slow loops only run every outer_every steps, the fast ones every step
  if (c.tick++ % g.outer_every == 0) outer_loop(c, P, s, sp, dt * g.outer_every);
  // attitude error seen from the body, flipped if needed so it turns the short way
  Quat e = conj(s.q) * c.att_sp;
  if (e.w < 0) e = e * -1;
  // small angle: twice the quaternion's vector part is the rotation still to go
  Vec3 w_sp = clamp3(mul(g.att, Vec3{e.x, e.y, e.z} * 2), g.max_rate);
  // rate loop: spin rate error becomes angular acceleration, then torque through the inertia
  Vec3 we = w_sp - s.w;
  double m = g.rate_i_max;
  c.rate_int = clamp3(c.rate_int + mul(g.rate_i, we) * dt, {m, m, m});
  Vec3 torque = mul(P.inertia, mul(g.rate_p, we) + c.rate_int);
  // collective thrust is the wanted force along where body z points right now, never under the floor,
  // since with no thrust the mixer has nothing to steer with and a flipped drone could never right itself
  double floor = g.thrust_min * 4 * P.k_f * P.max_rotor_speed * P.max_rotor_speed;
  double thrust = std::max(dot(c.force_sp, rotate(s.q, {0, 0, 1})), floor);
  return mix(P, thrust, torque);
}
