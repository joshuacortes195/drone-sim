#include <emscripten/emscripten.h>

#include <cmath>

#include "controller.h"
#include "quad.h"

// the one drone the page flies, its flight controller and where it's told to go
static QuadParams P;
static QuadState s, prev;
static Controller c;
static Setpoint sp;

// the physics step, same 1 ms the controller was tuned on
static const double DT = 0.001;

// the last two states packed as p, v, q (w x y z), body rates, rotor speeds, 17 numbers each
// the page draws the drone part way between them
static double now_buf[17], prev_buf[17];

// copies a state into one of the buffers above
static void pack(const QuadState& st, double* out) {
  const double v[17] = {st.p.x, st.p.y, st.p.z, st.v.x, st.v.y, st.v.z, st.q.w, st.q.x, st.q.y, st.q.z,
                        st.w.x, st.w.y, st.w.z, st.rotor[0], st.rotor[1], st.rotor[2], st.rotor[3]};
  for (int i = 0; i < 17; i++) out[i] = v[i];
}

extern "C" {

// puts the drone hovering at a spot facing yaw, with the rotors already at hover speed and a fresh controller
EMSCRIPTEN_KEEPALIVE void reset(double x, double y, double z, double yaw) {
  s = QuadState{};
  s.p = {x, y, z};
  s.q = axis_angle({0, 0, 1}, yaw);
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  s.rotor = {w, w, w, w};
  c = Controller{};
  sp = Setpoint{};
  sp.pos = s.p;
  sp.yaw = yaw;
  prev = s;
  pack(s, now_buf);
  pack(prev, prev_buf);
}

// tells the controller where to fly and which way to face, it holds there once it arrives
EMSCRIPTEN_KEEPALIVE void set_target(double x, double y, double z, double yaw) {
  sp.pos = {x, y, z};
  sp.yaw = yaw;
}

// runs the controller and physics forward a few 1 ms steps, so js only crosses into wasm once a frame
EMSCRIPTEN_KEEPALIVE void advance(int steps) {
  for (int i = 0; i < steps; i++) {
    prev = s;
    s = step(P, s, control(c, P, s, sp, DT), DT);
  }
  pack(s, now_buf);
  pack(prev, prev_buf);
}

// where the two state buffers sit in wasm memory, so js can read them in place
EMSCRIPTEN_KEEPALIVE double* state_now() { return now_buf; }
EMSCRIPTEN_KEEPALIVE double* state_prev() { return prev_buf; }

// where rotor i sits on the body, so the 3d model matches the physics instead of copying numbers
EMSCRIPTEN_KEEPALIVE double rotor_pos(int i, int axis) {
  const Vec3& r = P.rotor_pos[i];
  return axis == 0 ? r.x : axis == 1 ? r.y : r.z;
}

// the physics step in seconds, so the page's clock uses the same one
EMSCRIPTEN_KEEPALIVE double step_size() { return DT; }
}
