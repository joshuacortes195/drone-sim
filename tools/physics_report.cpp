#include "quad.h"

#include <chrono>
#include <cmath>
#include <cstdio>

// prints the numbers that go in results/physics.md

static const QuadParams P;
static const Rotors OFF = {0, 0, 0, 0};

// semi implicit euler for comparison: update the rates first, then move with the new rates
static QuadState euler_step(const QuadState& s, double dt) {
  QuadState d = derivative(P, s, OFF);
  QuadState n = s;
  n.v = s.v + d.v * dt;
  n.w = s.w + d.w * dt;
  n.p = s.p + n.v * dt;
  n.q = normalized(s.q + s.q * Quat{0, n.w.x, n.w.y, n.w.z} * (0.5 * dt));
  return n;
}

// spin the world sees and the rotational energy, both should never change with no torque
static Vec3 momentum(const QuadState& s) { return rotate(s.q, mul(P.inertia, s.w)); }
static double energy(const QuadState& s) { return 0.5 * dot(s.w, mul(P.inertia, s.w)); }

// tumbles from w = (1, 2, 3) for 60 s and reports the worst relative drift seen
static void tumble(const char* name, bool rk4) {
  QuadState s;
  s.w = {1, 2, 3};
  Vec3 L0 = momentum(s);
  double E0 = energy(s), worst_L = 0, worst_E = 0;
  for (int i = 0; i < 60000; i++) {
    s = rk4 ? step(P, s, OFF, 0.001) : euler_step(s, 0.001);
    worst_L = std::fmax(worst_L, norm(momentum(s) - L0) / norm(L0));
    worst_E = std::fmax(worst_E, std::abs(energy(s) - E0) / E0);
  }
  std::printf("%-22s worst momentum drift %.3e   worst energy drift %.3e\n", name, worst_L, worst_E);
}

int main() {
  // free fall from rest at 100 m for 4 s, compared to z0 - g t^2 / 2
  QuadState s;
  s.p = {0, 0, 100};
  for (int i = 0; i < 4000; i++) s = step(P, s, OFF, 0.001);
  double exact = 100 - 0.5 * 9.81 * 16;
  std::printf("free fall 4 s (rk4)    z error %.3e m   vz error %.3e m/s\n", s.p.z - exact,
              s.v.z + 9.81 * 4);

  tumble("tumble 60 s, rk4", true);
  tumble("tumble 60 s, euler", false);

  // cost of one rk4 step at hover, averaged over a million steps
  QuadState h;
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  h.rotor = {w, w, w, w};
  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < 1000000; i++) h = step(P, h, {w, w, w, w}, 0.001);
  double ns = std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - t0).count() / 1e6;
  std::printf("rk4 step cost          %.1f ns per step (z %.3f so the loop isn't optimized away)\n", ns, h.p.z);
}
