// prints the hover in wind numbers for results/wind.md
#include "controller.h"
#include "estimator.h"
#include "quad.h"
#include "sensors.h"
#include "wind.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

// what the controller flies on
enum Nav { TRUTH, RTK, EVERYDAY };

// how one hover went over the measured minute
struct Hover {
  double rms = 0, worst = 0, lean = 0, max_tilt = 0, est_rms = 0, lowest = 1e9;
};

// holds a hover at 10 m in the wind for 20 s to settle, then measures the next 60 s
static Hover hover(double breeze, double gusts, Nav nav, unsigned seed) {
  QuadParams P;
  Controller c;
  Setpoint sp;
  sp.pos = {0, 0, 10};
  QuadState s;
  s.p = sp.pos;
  double w = std::sqrt(P.mass * 9.81 / (4 * P.k_f));
  s.rotor = {w, w, w, w};

  // breeze from the west like the page, gusts on all three axes
  Wind wnd = make_wind({breeze, 0, 0}, gusts, seed);

  // sensors and estimator, the gyro calibrated for 1 s first like the page and the native flights
  Noise n = nav == EVERYDAY ? Noise{} : rtk();
  Sensors sn = make_sensors(n, seed + 5000);
  Estimator est = make_estimator(s, n);
  Vec3 gsum;
  for (int i = 0; i < 1000; i++) gsum = gsum + read_imu(sn, P, s, 0.001).gyro;
  est.gyro_bias = gsum * 0.001;

  Hover h;
  double sum = 0, est_sum = 0, lean_sum = 0;
  const int settle = 20000, measured = 60000;
  for (int i = 1; i <= settle + measured; i++) {
    // the wind this step, felt by the imu and the physics alike
    Vec3 air = wind_step(wnd, 0.001);
    predict(est, read_imu(sn, P, s, 0.001, air), 0.001);
    if (i % 100 == 0) {
      gps(est, read_gps(sn, s));
      mag(est, read_mag(sn, s));
    }
    QuadState e = estimate(est);
    s = step(P, s, control(c, P, nav == TRUTH ? s : e, sp, 0.001), 0.001, air);

    // only the minute after settling counts
    if (i <= settle) continue;
    double off = norm(s.p - sp.pos), tilt = std::acos(std::min(1.0, rotate(s.q, {0, 0, 1}).z));
    sum += off * off;
    h.worst = std::max(h.worst, off);
    h.max_tilt = std::max(h.max_tilt, tilt);
    lean_sum += tilt;
    est_sum += norm(e.p - s.p) * norm(e.p - s.p);
    h.lowest = std::min(h.lowest, s.p.z);
  }
  h.rms = std::sqrt(sum / measured);
  h.est_rms = std::sqrt(est_sum / measured);
  h.lean = lean_sum / measured;
  return h;
}

// median and worst of a list
static double median(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}
static double worst(const std::vector<double>& v) { return *std::max_element(v.begin(), v.end()); }

int main() {
  const int seeds = 20;
  const double deg = 180 / 3.14159265358979;
  const char* names[] = {"truth", "rtk", "everyday gps"};
  std::printf("| flies on | breeze m/s | gusts m/s | off its spot rms, median / worst m | furthest off, median / worst m "
              "| mean lean deg | most tilt deg | estimate off rms, median m | lowest m |\n");
  std::printf("|---|---|---|---|---|---|---|---|---|\n");
  for (Nav nav : {TRUTH, RTK, EVERYDAY})
    for (double breeze : {0.0, 4.0, 8.0})
      for (double gusts : {0.0, 1.5, 3.0}) {
        std::vector<double> rms, far, lean, tilt, est;
        double lowest = 1e9;
        for (int seed = 0; seed < seeds; seed++) {
          Hover h = hover(breeze, gusts, nav, unsigned(seed));
          rms.push_back(h.rms);
          far.push_back(h.worst);
          lean.push_back(h.lean * deg);
          tilt.push_back(h.max_tilt * deg);
          est.push_back(h.est_rms);
          lowest = std::min(lowest, h.lowest);
        }
        std::printf("| %s | %.0f | %.1f | %.4f / %.4f | %.4f / %.4f | %.2f | %.2f | %.4f | %.2f |\n", names[nav], breeze,
                    gusts, median(rms), worst(rms), median(far), worst(far), median(lean), worst(tilt), median(est),
                    lowest);
      }
}
