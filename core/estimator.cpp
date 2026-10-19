#include "estimator.h"

#include <algorithm>
#include <cmath>

// complementary filter gains
static const double kTilt = 0.03;     // rad/s of correction per rad of tilt error, from the accelerometer. weak on purpose
static const double kTiltI = 0.00046; // how fast tilt error turns into learned gyro bias, (kTilt / 1.4)^2 for 0.7 damping
static const double kHeading = 0.05;  // share of the heading error fixed at each magnetometer reading
static const double kHeadingI = 0.002;  // share of it that goes into the learned gyro bias

// smallest noise the kalman filter assumes. the accel one is well above the bmi088's 1.57e-3
// to cover what the model gets wrong (step timing, the tilt pull), see results/estimation.md
static const double kMinDensity = 1e-2, kMinWalk = 1e-5, kMinGps = 0.01;

// 9 by 9 matrix helpers for the kalman filter, row major
using Mat9 = double[9][9];

static void mul(const Mat9 a, const Mat9 b, Mat9 out) {
  for (int i = 0; i < 9; i++)
    for (int j = 0; j < 9; j++) {
      double sum = 0;
      for (int k = 0; k < 9; k++) sum += a[i][k] * b[k][j];
      out[i][j] = sum;
    }
}

// the body to world rotation as a 3 by 3 matrix, columns are the body axes in the world
static void rotation(Quat q, double R[3][3]) {
  Vec3 cols[3] = {rotate(q, {1, 0, 0}), rotate(q, {0, 1, 0}), rotate(q, {0, 0, 1})};
  for (int j = 0; j < 3; j++) {
    R[0][j] = cols[j].x;
    R[1][j] = cols[j].y;
    R[2][j] = cols[j].z;
  }
}

Estimator make_estimator(const QuadState& s, const Noise& n) {
  Estimator e{};
  e.p = s.p;
  e.v = s.v;
  e.q = s.q;
  e.n = n;
  // starts sure of where it is, unsure about the accelerometer's turn on offset
  double ab = std::max(n.accel_bias0, 0.01);
  for (int i = 0; i < 3; i++) {
    e.P[i][i] = 0.01;
    e.P[3 + i][3 + i] = 0.01;
    e.P[6 + i][6 + i] = ab * ab;
  }
  return e;
}

void predict(Estimator& e, const Imu& m, double dt) {
  e.w = m.gyro - e.gyro_bias;

  // tilt check: in a steady hover the accelerometer points up the same way gravity does.
  // only trusted within 3% of 1 g, since speeding up or turning hard bends that reading away from true up.
  // uses the raw reading, not the kalman filter's bias guess, or the two filters trade a tilt error back and forth
  double fn = norm(m.accel);
  double trust = std::max(0.0, 1 - std::abs(fn / 9.81 - 1) / 0.03);
  Vec3 err;
  if (trust > 0) err = cross(m.accel * (1 / fn), rotate(conj(e.q), {0, 0, 1})) * trust;

  // a steady tilt error means the gyro is off, so some of it is learned as gyro bias
  e.gyro_bias = e.gyro_bias - err * (kTiltI * dt);

  // turns the attitude by the gyro plus the tilt correction
  Vec3 turn = e.w + err * kTilt;
  double rate = norm(turn);
  if (rate > 0) e.q = normalized(e.q * axis_angle(turn * (1 / rate), rate * dt));

  // the accelerometer's push minus its learned bias, turned into the world, plus gravity, moves the position and velocity
  Vec3 f = m.accel - e.accel_bias;
  Vec3 a = rotate(e.q, f) + Vec3{0, 0, -9.81};
  e.p = e.p + e.v * dt + a * (dt * dt / 2);
  e.v = e.v + a * dt;

  // how the uncertainty spreads: F is how each state feeds the others over dt
  double R[3][3];
  rotation(e.q, R);
  Mat9 F = {};
  for (int i = 0; i < 9; i++) F[i][i] = 1;
  for (int i = 0; i < 3; i++) {
    F[i][3 + i] = dt;  // velocity moves position
    for (int j = 0; j < 3; j++) {
      F[i][6 + j] = -R[i][j] * dt * dt / 2;  // a wrong bias pushes position
      F[3 + i][6 + j] = -R[i][j] * dt;       // and velocity
    }
  }
  Mat9 FP, FPFt, Ft;
  for (int i = 0; i < 9; i++)
    for (int j = 0; j < 9; j++) Ft[i][j] = F[j][i];
  mul(F, e.P, FP);
  mul(FP, Ft, FPFt);

  // plus the new uncertainty this step adds: accel noise into velocity, bias wander into the bias
  Vec3 d = e.n.accel_density;
  double d2[3] = {std::pow(std::max(d.x, kMinDensity), 2), std::pow(std::max(d.y, kMinDensity), 2),
                  std::pow(std::max(d.z, kMinDensity), 2)};
  double walk = std::max(e.n.accel_walk, kMinWalk);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      double q = 0;
      for (int k = 0; k < 3; k++) q += R[i][k] * d2[k] * R[j][k];
      FPFt[3 + i][3 + j] += q * dt;
    }
    FPFt[6 + i][6 + i] += walk * walk * dt;
  }
  for (int i = 0; i < 9; i++)
    for (int j = 0; j < 9; j++) e.P[i][j] = FPFt[i][j];
}

void gps(Estimator& e, Vec3 fix) {
  // how much the fix and the guess disagree, and how unsure that disagreement is: S = P_pos + gps noise
  double xy = std::max(e.n.gps_xy, kMinGps), z = std::max(e.n.gps_z, kMinGps);
  double S[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) S[i][j] = e.P[i][j];
  S[0][0] += xy * xy;
  S[1][1] += xy * xy;
  S[2][2] += z * z;

  // S inverse by cofactors
  double det = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1]) - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0]) +
               S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
  double Si[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      int a = (j + 1) % 3, b = (j + 2) % 3, c = (i + 1) % 3, d = (i + 2) % 3;
      Si[i][j] = (S[a][c] * S[b][d] - S[a][d] * S[b][c]) / det;
    }

  // kalman gain: how far each of the 9 numbers moves per meter of disagreement
  double K[9][3];
  for (int i = 0; i < 9; i++)
    for (int j = 0; j < 3; j++) {
      K[i][j] = 0;
      for (int k = 0; k < 3; k++) K[i][j] += e.P[i][k] * Si[k][j];
    }

  // moves the guess toward the fix
  Vec3 r = fix - e.p;
  double x[9] = {e.p.x, e.p.y, e.p.z, e.v.x, e.v.y, e.v.z, e.accel_bias.x, e.accel_bias.y, e.accel_bias.z};
  for (int i = 0; i < 9; i++) x[i] += K[i][0] * r.x + K[i][1] * r.y + K[i][2] * r.z;
  e.p = {x[0], x[1], x[2]};
  e.v = {x[3], x[4], x[5]};
  e.accel_bias = {x[6], x[7], x[8]};

  // and it's now surer: P = P - K * (top three rows of P), kept symmetric
  Mat9 Pn;
  for (int i = 0; i < 9; i++)
    for (int j = 0; j < 9; j++) Pn[i][j] = e.P[i][j] - (K[i][0] * e.P[0][j] + K[i][1] * e.P[1][j] + K[i][2] * e.P[2][j]);
  for (int i = 0; i < 9; i++)
    for (int j = 0; j < 9; j++) e.P[i][j] = (Pn[i][j] + Pn[j][i]) / 2;
}

void mag(Estimator& e, double heading_read) {
  // heading error, the short way round
  double err = heading_read - heading(e.q);
  err = std::atan2(std::sin(err), std::cos(err));
  // turns the attitude part of the way about world up, and learns a bit of it as gyro bias
  e.q = normalized(axis_angle({0, 0, 1}, kHeading * err) * e.q);
  e.gyro_bias = e.gyro_bias - rotate(conj(e.q), {0, 0, kHeadingI * err});
}

QuadState estimate(const Estimator& e) {
  QuadState s;
  s.p = e.p;
  s.v = e.v;
  s.q = e.q;
  s.w = e.w;
  return s;
}
