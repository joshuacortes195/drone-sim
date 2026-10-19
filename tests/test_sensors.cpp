#include "doctest.h"
#include "sensors.h"

#include <cmath>
#include <vector>

static const QuadParams P;

// rotor speed where all four together hold up the drone's weight
static double hover_speed() { return std::sqrt(P.mass * 9.81 / (4 * P.k_f)); }

// sensors with every noise and bias turned off
static Sensors perfect() { return make_sensors(quiet(), 1); }

TEST_CASE("a perfect accelerometer reads 9.81 up in hover and zero in free fall") {
  Sensors sn = perfect();
  QuadState s;
  double w = hover_speed();
  s.rotor = {w, w, w, w};
  Imu hover = read_imu(sn, P, s, 0.001);
  CHECK(hover.accel.x == doctest::Approx(0).epsilon(1e-12));
  CHECK(hover.accel.y == doctest::Approx(0).epsilon(1e-12));
  CHECK(hover.accel.z == doctest::Approx(9.81).epsilon(1e-9));

  // motors off and falling, nothing but gravity acts so the accelerometer feels nothing
  s.rotor = {0, 0, 0, 0};
  s.v = {0, 0, -5};
  Imu fall = read_imu(sn, P, s, 0.001);
  CHECK(norm(fall.accel) == doctest::Approx(0).epsilon(1e-12));
}

TEST_CASE("a perfect accelerometer reads in the body frame, tipped 90 degrees gravity's push shows on body x") {
  // nose pitched straight down (rotated +90 about y), rotors at hover: thrust points along world +x
  Sensors sn = perfect();
  QuadState s;
  double w = hover_speed();
  s.rotor = {w, w, w, w};
  s.q = axis_angle({0, 1, 0}, 3.14159265358979323846 / 2);
  Imu m = read_imu(sn, P, s, 0.001);
  // thrust is fixed to the body, so it still reads straight up the body z axis
  CHECK(m.accel.z == doctest::Approx(9.81).epsilon(1e-9));
  CHECK(std::abs(m.accel.x) < 1e-9);
}

TEST_CASE("a perfect gyro reads the body spin rate") {
  Sensors sn = perfect();
  QuadState s;
  s.w = {0.3, -0.2, 1.1};
  Imu m = read_imu(sn, P, s, 0.001);
  CHECK(m.gyro.x == doctest::Approx(0.3));
  CHECK(m.gyro.y == doctest::Approx(-0.2));
  CHECK(m.gyro.z == doctest::Approx(1.1));
}

// spread of a list of numbers around its own average
static double stddev(const std::vector<double>& xs) {
  double mean = 0, var = 0;
  for (double x : xs) mean += x;
  mean /= xs.size();
  for (double x : xs) var += (x - mean) * (x - mean);
  return std::sqrt(var / (xs.size() - 1));
}

// true when a is within a fraction rel of b. doctest's approx adds 1 to the scale, too loose for small numbers
static bool within(double a, double b, double rel) { return std::abs(a - b) <= rel * std::abs(b); }

TEST_CASE("imu white noise matches the bmi088 datasheet at 1000 samples a second") {
  // only white noise, no biases, so the spread is just the noise
  Noise n;
  n.accel_bias0 = n.gyro_bias0 = n.accel_walk = n.gyro_walk = 0;
  Sensors sn = make_sensors(n, 7);
  QuadState s;
  std::vector<double> ax, az, gy;
  for (int i = 0; i < 20000; i++) {
    Imu m = read_imu(sn, P, s, 0.001);
    ax.push_back(m.accel.x);
    az.push_back(m.accel.z);
    gy.push_back(m.gyro.y);
  }
  // 160 and 190 ug/sqrt(Hz) and 0.014 deg/s/sqrt(Hz) times sqrt(1000 Hz)
  CHECK(within(stddev(ax), 0.04964, 0.03));
  CHECK(within(stddev(az), 0.05894, 0.03));
  CHECK(within(stddev(gy), 7.727e-3, 0.03));
}

TEST_CASE("turn on biases spread like the datasheet offsets and the readings sit on them") {
  // only turn on biases, no white noise or wander
  Noise n = quiet();
  n.accel_bias0 = 0.196;
  n.gyro_bias0 = 0.01745;
  std::vector<double> ab, gb;
  for (unsigned seed = 1; seed <= 2000; seed++) {
    Sensors sn = make_sensors(n, seed);
    ab.push_back(sn.accel_bias.y);
    gb.push_back(sn.gyro_bias.z);
  }
  CHECK(within(stddev(ab), 0.196, 0.05));
  CHECK(within(stddev(gb), 0.01745, 0.05));

  // a still drone's reading is the truth plus the bias
  Sensors sn = make_sensors(n, 3);
  QuadState s;
  Imu m = read_imu(sn, P, s, 0.001);
  CHECK(m.accel.x == doctest::Approx(sn.accel_bias.x));
  CHECK(m.gyro.z == doctest::Approx(sn.gyro_bias.z));
  CHECK(std::abs(sn.gyro_bias.z) > 0);
}

TEST_CASE("biases wander as a random walk, spread grows with the square root of time") {
  Noise n = quiet();
  n.accel_walk = 6e-3;
  n.gyro_walk = 3.88e-5;
  std::vector<double> ab, gb;
  QuadState s;
  for (unsigned seed = 1; seed <= 300; seed++) {
    Sensors sn = make_sensors(n, seed);
    for (int i = 0; i < 4000; i++) read_imu(sn, P, s, 0.001);
    ab.push_back(sn.accel_bias.x);
    gb.push_back(sn.gyro_bias.x);
  }
  // after 4 s: walk * sqrt(4)
  CHECK(within(stddev(ab), 0.012, 0.1));
  CHECK(within(stddev(gb), 7.76e-5, 0.1));
}

TEST_CASE("gps spreads 2.12 m across and 4.24 m up and down around the true position, centered on it") {
  Sensors sn = make_sensors(Noise{}, 11);
  QuadState s;
  s.p = {3, -4, 10};
  std::vector<double> x, z;
  double mean_x = 0;
  for (int i = 0; i < 20000; i++) {
    Vec3 g = read_gps(sn, s);
    x.push_back(g.x);
    z.push_back(g.z);
    mean_x += g.x / 20000;
  }
  CHECK(within(stddev(x), 2.12, 0.03));
  CHECK(within(stddev(z), 4.24, 0.03));
  CHECK(std::abs(mean_x - 3) < 0.05);
}

TEST_CASE("magnetometer reads the true heading with 0.015 rad of noise, even tilted") {
  Sensors sn = make_sensors(Noise{}, 12);
  QuadState s;
  // facing 2 rad, then rolled 0.4 rad, which mustn't change the heading
  s.q = axis_angle({0, 0, 1}, 2.0) * axis_angle({1, 0, 0}, 0.4);
  std::vector<double> h;
  for (int i = 0; i < 20000; i++) h.push_back(read_mag(sn, s));
  CHECK(within(stddev(h), 0.015, 0.03));
  double mean = 0;
  for (double v : h) mean += v / h.size();
  CHECK(std::abs(mean - 2.0) < 1e-3);

  // a perfect one facing just past south (-pi) stays wrapped to -pi..pi
  Sensors p = perfect();
  s.q = axis_angle({0, 0, 1}, -3.1);
  CHECK(read_mag(p, s) == doctest::Approx(-3.1));
}
