#pragma once

#include "quad.h"

#include <random>

// how noisy each sensor is. defaults are real parts, sources in docs/plan.md:
// bosch bmi088 imu, u-blox neo-m8n gps, isentek ist8310 magnetometer
struct Noise {
  Vec3 accel_density = {1.5696e-3, 1.5696e-3, 1.8639e-3};  // m/s^2 per sqrt(Hz), 160 and 190 ug/sqrt(Hz)
  double gyro_density = 2.4435e-4;     // rad/s per sqrt(Hz), 0.014 deg/s/sqrt(Hz)
  double accel_bias0 = 0.196;          // m/s^2, typical turn on offset (20 mg)
  double gyro_bias0 = 0.01745;         // rad/s, turn on offset (1 deg/s)
  double accel_walk = 6e-3;            // m/s^3 per sqrt(Hz), how fast the accel bias wanders (rotors adis16448 default)
  double gyro_walk = 3.88e-5;          // rad/s^2 per sqrt(Hz), same for the gyro
  double gps_xy = 2.12, gps_z = 4.24;  // m per axis, 2.5 m cep50 / 1.1774, vertical twice that like px4 sitl
  double mag = 0.015;                  // rad of heading, 0.3 uT over a 20 uT horizontal field
};

// every noise and bias set to zero, the sensors read the truth
Noise quiet();

// the sensors' own random numbers and the biases they've drifted to so far
struct Sensors {
  Noise n;
  std::mt19937 rng;
  Vec3 accel_bias, gyro_bias;
};

// sets up the sensors with a seed and draws their turn on biases
Sensors make_sensors(const Noise& n, unsigned seed);

// one imu sample in the body frame: accel is every push except gravity per kg, gyro is the spin rate
struct Imu {
  Vec3 accel, gyro;
};

// reads the imu, dt is the time since the last read and sets how far the biases wander
Imu read_imu(Sensors& sn, const QuadParams& P, const QuadState& s, double dt);

// a noisy world position from the gps
Vec3 read_gps(Sensors& sn, const QuadState& s);

// a noisy heading from the magnetometer, radians from east toward north
double read_mag(Sensors& sn, const QuadState& s);

// the same imu and magnetometer with an rtk gps instead (u-blox zed-f9p): 0.01 m + 1 ppm cep horizontal
// and median vertical on a 1 km baseline, 10 Hz with gps + glonass + galileo
Noise rtk();
