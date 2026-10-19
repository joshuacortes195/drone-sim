#pragma once

#include "sensors.h"

// what the drone thinks its state is, built from the sensors alone.
// position, velocity and accelerometer bias come from a kalman filter (imu predicts, gps corrects),
// attitude from a complementary filter (gyro, a weak tilt pull from the accelerometer, heading from the magnetometer)
struct Estimator {
  Vec3 p, v, accel_bias;  // the kalman filter's 9 numbers
  double P[9][9];         // how unsure it is about each of them and how they're linked
  Quat q;                 // attitude
  Vec3 w;                 // spin rate with the gyro bias taken out
  Vec3 gyro_bias;         // learned by the complementary filter
  Noise n;                // sensor noise the filter assumes
};

// starts the estimator at a known state, like sitting on the launch pad
Estimator make_estimator(const QuadState& s, const Noise& n);

// moves the estimate forward dt seconds with one imu sample
void predict(Estimator& e, const Imu& m, double dt);

// pulls the estimate toward a gps fix
void gps(Estimator& e, Vec3 fix);

// pulls the heading toward a magnetometer reading
void mag(Estimator& e, double heading);

// the estimate packed as a state the controller can fly on
QuadState estimate(const Estimator& e);
