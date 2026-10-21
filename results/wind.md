# Wind results (phase 6a)

Measured 2026-10-21 on my PC with `tools/wind_report.cpp`.

## How

- machine: AMD Ryzen 7 3700X (8 cores, 16 threads), 16 GB RAM, Windows 10 Home 19045
- build: MSVC 19.44 (Visual Studio 2022 Build Tools), CMake Release build (`/O2 /Ob2 /DNDEBUG`), C++17, double precision
- the drone holds a hover at 10 m over open ground, no buildings. It starts level at rotor hover speed with the wind already blowing, gets 20 s to settle, then the next 60 s are measured
- wind: a steady breeze from the west (blowing toward +x) of 0, 4 or 8 m/s, plus gauss markov gusts on all three axes with a spread (σ) of 0, 1.5 or 3 m/s and tau 2 s (`core/wind.{h,cpp}`). Those are the page's slider ranges. The wind only reaches the drone through rotor drag and the rolling moment (see `docs/plan.md`)
- flown three ways: on the true state, on the estimate with the RTK GPS (ZED-F9P, the page's default) and on the estimate with the everyday GPS (NEO-M8N). Same IMU and magnetometer in all three, same as `results/estimation.md`. On the truth the RTK estimator still runs alongside, it just isn't flown on
- seeds: 20 per row, 0 to 19 for the gusts and seed + 5000 for the sensor noise. 27 rows, 540 hovers
- controller: the gains in `results/control.md`, unchanged. Physics, controller and IMU at 1 ms, GPS and magnetometer at 10 Hz, gyro calibrated for 1 s first
- off its spot is the 3D distance from the true center to the set point. Lean is the angle between the body's up axis and straight up. Lowest is the lowest altitude any of the 20 hovers reached
- run it: build, then `build/wind_report.exe` (about 90 s for all 540)

## Results

| flies on | breeze m/s | gusts m/s | off its spot rms, median / worst m | furthest off, median / worst m | mean lean deg | most tilt deg | estimate off rms, median m | lowest m |
|---|---|---|---|---|---|---|---|---|
| truth | 0 | 0.0 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 0.00 | 0.00 | 0.0105 | 10.00 |
| truth | 0 | 1.5 | 0.0741 / 0.0928 | 0.1696 / 0.2075 | 1.77 | 5.36 | 0.0106 | 10.00 |
| truth | 0 | 3.0 | 0.1498 / 0.1834 | 0.3436 / 0.4096 | 3.56 | 10.89 | 0.0109 | 9.99 |
| truth | 4 | 0.0 | 0.0000 / 0.0000 | 0.0001 / 0.0001 | 3.27 | 3.27 | 0.0105 | 10.00 |
| truth | 4 | 1.5 | 0.0736 / 0.0927 | 0.1698 / 0.2053 | 3.68 | 8.33 | 0.0106 | 9.99 |
| truth | 4 | 3.0 | 0.1485 / 0.1834 | 0.3413 / 0.4023 | 4.68 | 14.02 | 0.0109 | 9.98 |
| truth | 8 | 0.0 | 0.0000 / 0.0000 | 0.0003 / 0.0003 | 6.50 | 6.50 | 0.0105 | 10.00 |
| truth | 8 | 1.5 | 0.0722 / 0.0915 | 0.1691 / 0.2009 | 6.72 | 11.65 | 0.0106 | 9.98 |
| truth | 8 | 3.0 | 0.1456 / 0.1814 | 0.3324 / 0.3936 | 7.28 | 17.37 | 0.0109 | 9.96 |
| rtk | 0 | 0.0 | 0.0194 / 0.0259 | 0.0374 / 0.0496 | 0.23 | 0.80 | 0.0105 | 9.97 |
| rtk | 0 | 1.5 | 0.0776 / 0.0951 | 0.1819 / 0.2196 | 1.80 | 5.52 | 0.0107 | 9.97 |
| rtk | 0 | 3.0 | 0.1514 / 0.1849 | 0.3525 / 0.4236 | 3.57 | 11.07 | 0.0109 | 9.96 |
| rtk | 4 | 0.0 | 0.0195 / 0.0261 | 0.0376 / 0.0501 | 3.27 | 3.98 | 0.0105 | 9.97 |
| rtk | 4 | 1.5 | 0.0768 / 0.0953 | 0.1779 / 0.2168 | 3.69 | 8.52 | 0.0107 | 9.96 |
| rtk | 4 | 3.0 | 0.1498 / 0.1852 | 0.3529 / 0.4154 | 4.68 | 13.73 | 0.0109 | 9.95 |
| rtk | 8 | 0.0 | 0.0196 / 0.0263 | 0.0379 / 0.0506 | 6.50 | 7.20 | 0.0106 | 9.97 |
| rtk | 8 | 1.5 | 0.0751 / 0.0945 | 0.1736 / 0.2118 | 6.72 | 11.68 | 0.0106 | 9.96 |
| rtk | 8 | 3.0 | 0.1466 / 0.1836 | 0.3463 / 0.4040 | 7.28 | 17.10 | 0.0109 | 9.94 |
| everyday gps | 0 | 0.0 | 1.2395 / 1.6963 | 2.2264 / 2.8522 | 3.71 | 14.58 | 1.0683 | 7.62 |
| everyday gps | 0 | 1.5 | 1.2438 / 1.6786 | 2.2508 / 2.8226 | 4.12 | 18.09 | 1.0704 | 7.65 |
| everyday gps | 0 | 3.0 | 1.2568 / 1.6734 | 2.2919 / 2.8329 | 5.14 | 23.13 | 1.0689 | 7.69 |
| everyday gps | 4 | 0.0 | 1.2512 / 1.7088 | 2.2278 / 2.8924 | 4.73 | 18.11 | 1.0599 | 7.64 |
| everyday gps | 4 | 1.5 | 1.2512 / 1.6878 | 2.2463 / 2.8577 | 5.06 | 16.55 | 1.0612 | 7.68 |
| everyday gps | 4 | 3.0 | 1.2508 / 1.6804 | 2.2848 / 2.8357 | 5.94 | 19.59 | 1.0644 | 7.72 |
| everyday gps | 8 | 0.0 | 1.2575 / 1.7238 | 2.2289 / 2.9341 | 7.26 | 21.72 | 1.0518 | 7.66 |
| everyday gps | 8 | 1.5 | 1.2603 / 1.7005 | 2.2447 / 2.8994 | 7.46 | 20.32 | 1.0521 | 7.71 |
| everyday gps | 8 | 3.0 | 1.2575 / 1.6943 | 2.2732 / 2.8696 | 8.03 | 22.92 | 1.0541 | 7.75 |

What I read from it:

- a steady breeze costs nothing once the velocity integral has learned it. In 8 m/s it leans 6.50 deg into the wind and sits within 0.3 mm of its spot
- gusts are what push it around, and it scales with the gust size, not the breeze: about 0.074 m rms per 1.5 m/s of gusts, with the furthest it got pushed 0.41 m on the truth (0.42 m on RTK) in 3 m/s gusts
- on RTK it flies almost exactly like on the truth. The estimate is about 1 cm off and adds about 2 cm of wander in calm air
- on the everyday GPS the GPS noise swamps the wind. It wanders about 1.25 m rms whatever the wind does, and sank as low as 7.62 m from a 10 m hover. That's the 4.24 m vertical GPS error, not the wind, and it's why the drone can hit the ground right after a reset from 1.5 m with the everyday GPS on

## Before and after the accelerometer fix

The first version of the sensors never gave the accelerometer the wind. `read_imu()` worked out the force with still air, so rotor drag from the wind was a real push the IMU didn't feel. The Kalman filter predicts with the IMU, so every gust looked like the drone wasn't accelerating when it was, and only the GPS could pull it back. I found it while writing this tool and fixed it (the test "a perfect accelerometer feels the wind"). Same tool, same seeds, the only change being the wind left out of the IMU:

| flies on | breeze m/s | gusts m/s | off its spot rms, median before / after m | furthest off, worst before / after m | estimate off rms, median before / after m |
|---|---|---|---|---|---|
| rtk | 8 | 0.0 | 0.0477 / 0.0196 | 0.1154 / 0.0506 | 0.0127 / 0.0106 |
| rtk | 0 | 1.5 | 0.2495 / 0.0776 | 0.6945 / 0.2196 | 0.0487 / 0.0107 |
| rtk | 0 | 3.0 | 0.5001 / 0.1514 | 1.3737 / 0.4236 | 0.0966 / 0.0109 |
| rtk | 8 | 3.0 | 0.4941 / 0.1466 | 1.3611 / 0.4040 | 0.0958 / 0.0109 |
| everyday gps | 8 | 0.0 | 1.5485 / 1.2575 | 3.3117 / 2.9341 | 1.2794 / 1.0518 |
| everyday gps | 0 | 1.5 | 3.3071 / 1.2438 | 8.9909 / 2.8226 | 2.4695 / 1.0704 |
| everyday gps | 0 | 3.0 | 6.2428 / 1.2568 | 17.4416 / 2.8329 | 4.4611 / 1.0689 |
| everyday gps | 8 | 3.0 | 6.1047 / 1.2575 | 17.2004 / 2.8696 | 4.3718 / 1.0541 |

With the everyday GPS and 3 m/s gusts the drone was getting pushed up to 17 m off its spot. That explains the browser runs in the first 6a session where everyday GPS plus gusts crashed within 10 s. The steady breeze rows show the bug too, but much less, since the Kalman filter soaks a steady push up as accelerometer bias.
