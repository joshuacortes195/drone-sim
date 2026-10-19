# Estimation results (phase 5a)

Measured 2026-10-18 on my PC with `tools/plan_report.cpp` and the estimator tests.

## How

- machine: AMD Ryzen 7 3700X (8 cores, 16 threads), 16 GB RAM, Windows 10 Home 19045
- build: MSVC 19.44 (Visual Studio 2022 Build Tools), CMake Release build (`/O2 /Ob2 /DNDEBUG`), C++17, double precision
- flights: the same 200 planned flights as `results/planning.md` (seeds 0 to 199, same maps, queries, planner, path follower and controller gains). Each one is flown three times: on the true state, on the estimate with a NEO-M8N GPS, and on the estimate with a ZED-F9P RTK GPS. Sensor noise for a flight is seeded with the map seed + 5000
- loop: 1 ms physics. Every step the IMU is read and the estimator predicts. Every 100 steps (10 Hz) a GPS fix and a magnetometer heading go in. Then the controller runs on the estimate and the physics steps
- start: the drone sits in a hover at the first path point. The estimator starts at the true state, and the gyro bias is set from 1 s of gyro readings averaged while still, like a flight controller's calibration at power up
- estimate error is the distance from the estimated to the true position, sampled at each GPS fix. RMS and worst are per flight. The table shows the median, 95th percentile and worst over the 200 flights
- run it: build, then `build/plan_report.exe`

## Sensors

| sensor | part | value used | source |
|---|---|---|---|
| accelerometer white noise | Bosch BMI088 | 160 µg/√Hz x and y, 190 µg/√Hz z | BMI088 data sheet BST-BMI088-DS000-19 Rev 1.9, Table 4 |
| accelerometer turn on bias | BMI088 | σ = 20 mg (0.196 m/s²) per axis | same, Table 4 (typical zero g offset) |
| gyro white noise | BMI088 | 0.014 deg/s/√Hz | same, Table 5 |
| gyro turn on bias | BMI088 | σ = 1 deg/s per axis | same, Table 5 (zero rate offset) |
| bias random walk | not in the BMI088 data sheet | accel 6e-3 m/s³/√Hz, gyro 3.88e-5 rad/s²/√Hz | RotorS IMU plugin defaults (`gazebo_imu_plugin.h`, ADIS16448). A better IMU than the BMI088, so this probably understates the drift |
| GPS | u-blox NEO-M8N | 2.5 m CEP50 → σ = 2.12 m per horizontal axis, 10 Hz | NEO-M8 (FW3) data sheet UBX-15031086 R14, Table 1 |
| GPS vertical | not in the NEO-M8N data sheet | σ = 4.24 m | PX4 SITL GPS plugin uses twice the horizontal noise |
| RTK GPS | u-blox ZED-F9P | 0.01 m + 1 ppm CEP horizontal and median vertical, at a 1 km baseline 0.011 m → σ = 0.0093 m per horizontal axis, 0.0163 m vertical, 10 Hz (GPS + GLONASS + Galileo) | ZED-F9P-04B data sheet UBX-21044850 R05, Tables 2 and 3 |
| magnetometer | iSentek IST8310 | 0.3 µT RMS over a 20 µT horizontal field → σ = 0.015 rad of heading | IST8310 data sheet v1.5, section 4.4. The data sheet labels 0.3 µT RMS as the zero gauss offset, and I use it as the noise. Hard and soft iron error and a steady offset aren't modeled, so real heading error would be worse |

Per sample noise is density × √(1000 Hz). GPS noise is fresh on every fix. Real GPS error drifts slowly and is correlated from fix to fix, so this is kinder than the real thing.

## Flying the 200 planned paths

| | on the true state | on the estimate, NEO-M8N | on the estimate, ZED-F9P RTK |
|---|---|---|---|
| flights within the drone's 0.356 m reach of a building or the ground | 0 | 51 | 0 |
| flights whose center went inside a building or the ground | 0 | 1 | 0 |
| closest any flight got | 0.680 m | -1.207 m | 0.679 m |
| worst to setpoint, median / p95 / worst | 0.283 / 0.401 / 0.430 m | 2.319 / 3.935 / 5.594 m | 0.283 / 0.416 / 0.466 m |
| off the goal 3 s after the path ends, median / p95 / worst | 0.086 / 0.100 / 0.112 m | 1.047 / 2.107 / 3.136 m | 0.083 / 0.118 / 0.138 m |
| estimate RMS error, median / p95 / worst | | 1.076 / 1.607 / 2.152 m | 0.011 / 0.013 / 0.015 m |
| estimate worst error, median / p95 / worst | | 2.267 / 3.372 / 4.371 m | 0.036 / 0.047 / 0.063 m |
| raw GPS RMS error (3D), median / p95 / worst | | 5.146 / 5.533 / 5.835 m | 0.021 / 0.022 / 0.023 m |

The Kalman filter cuts the position error about 5x with the NEO-M8N (1.08 m vs 5.15 m RMS at the median) and about 2x with RTK (0.011 m vs 0.021 m). The planner keeps paths 0.789 m from buildings, a margin set assuming the drone knows exactly where it is, and the streets are 4 m wide. With the NEO-M8N the drone is often 1 to 2 m wrong about its own position, so a quarter of the flights clip a building. With RTK the flights are as safe as flying on the truth. So I'm using RTK from here on (`rtk()`), and the NEO-M8N stays as the comparison. In the code `Noise{}` is still the NEO-M8N setup, and the page will use RTK when it starts flying on the estimate in phase 5b.

A* times move between runs of this tool (18.0 and 16.0 ms medians on two runs this session, 15.5 ms in `results/planning.md`). I haven't looked into the spread, so the planning times stay as recorded there.

## Tuning the estimator

These come from the tests in `tests/test_estimator.cpp`, which fly the drone on the estimate:

- maneuver: hover at 5 m, then a setpoint 5 m east, 3 m up and turned 1 rad for 8 s, then back for 8 s
- the real noise check adds 44 s of hover on top, seeds 1 to 4
- the tilt recovery check starts the estimate 0.05 rad off in roll in a still hover, flying on the true state for 100 s with perfect sensors

The tilt pull trusts the accelerometer only within 3% of 1 g. With the integral gain set to (kp / 1.4)² for 0.7 damping:

| tilt gain kp (rad/s per rad) | integral ki | worst position / attitude error, maneuver with perfect sensors | worst attitude error, real noise (4 seeds) | tilt error after 100 s, from 0.05 rad |
|---|---|---|---|---|
| 0.03 (chosen) | 0.00046 | 0.0020 m / 0.0042 rad | 0.010 to 0.031 rad | 0.0105 rad |
| 0.05 | 0.0013 | 0.0029 m / 0.0062 rad | 0.008 to 0.034 rad | 0.0056 rad |
| 0.08 | 0.0033 | 0.0043 m / 0.0089 rad | 0.008 to 0.039 rad | 0.0002 rad |

A stronger pull fixes a wrong tilt faster but leans the estimate more during hard acceleration, where the accelerometer can't see tilt. Of the three I tried I picked the weakest, which still brings a 0.05 rad error down to 0.0105 rad in 100 s. It's also the only one that keeps the perfect sensor maneuver under the 0.005 rad the test asks for. With real noise the worst attitude error is about what a sideways accelerometer bias over g gives (0.196 / 9.81 = 0.020 rad), because in a hover the two look the same.

Other results with 0.03, from the same tests: with a 1 deg/s yaw gyro bias that turns up after calibration, the heading ends 0.0015 rad off after 90 s and the learned bias is 0.0003 rad/s off. A 0.005 rad/s roll gyro bias turning up after calibration gets learned within 300 s (the test checks within 0.001 rad/s). Without the tilt integral the same test ended 0.71 rad off.

Two changes came from measuring:

- **The tilt pull reads the raw accelerometer.** My first version subtracted the Kalman filter's bias estimate first. Then the filter learned a tilt error as fake sideways bias within seconds, and the tilt pull saw nothing to fix. From 0.05 rad, the error was still 0.038 rad after 100 s. With the raw reading it's 0.010 rad
- **The 1 g check takes off the learned up and down accel bias.** The first version checked the raw reading, so a z bias over 0.29 m/s² (3% of g, about 13% of drones at the BMI088's typical offset) switched the tilt pull off for the whole flight. The phase review caught it, and a test covers it now
- **The Kalman filter assumes 0.01 m/s²/√Hz of accelerometer noise**, not the BMI088's 0.0016, to cover model error (one IMU sample per step, the tilt pull). Measured with an earlier tilt setting (gain 0.02, integral 0.002, bias corrected reading): with perfect sensors, the worst position error through the maneuver went from 0.115 m to 0.010 m. With real noise, the RMS error on seeds 1 to 4 changed by less than 1%

The filter's GPS noise floor is 1 mm, under the RTK's 9.3 mm, so the filter uses the RTK numbers as written. Lowering it from 1 cm took the perfect sensor maneuver's worst position error from 0.010 m to 0.002 m.

Mutation check: seven deliberate bugs in `core/estimator.cpp` were each caught by at least one estimator test. The bugs were no heading integral, GPS fixes ignored, the tilt pull reversed, no tilt pull, the wrong sign on the bias coupling in F, and the gyro bias not subtracted, and no tilt integral (caught by the roll gyro bias test added after review).
