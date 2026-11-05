# Batch evaluation (phase 7)

Measured 2026-11-04 on my Intel Mac with `tools/batch_eval.cpp`. The headline run is meant for my PC like phases 4 to 6a, so this table gets rerun there and both kept, labeled.

## How

- machine: Intel Mac, Core i5-8259U @ 2.30GHz (4 cores, 8 threads), 8 GB RAM, macOS 15.7.7
- build: Apple clang 17.0.0, CMake Release build (`-O3 -DNDEBUG`), C++17, double precision, one thread
- maps: seeds 0 to 999, one seeded city each from `city()` in `core/map.cpp` (5 x 5 lots, 8 m lots on a 12 m grid, buildings 4 to 18 m tall, 20% of lots empty)
- trips: one per city. Start and goal are random free cells of the planning grid (0.5 m voxels, buildings grown by 0.79 m) at least 20 m apart, from their own random stream (`seed + 20000`), so they aren't the trips in `results/planning.md` or `results/lidar.md`
- planning: 26 neighbor A* plus line of sight smoothing, timed once per trip
- flying: the full stack, the same as the page. Physics on a fixed 1 ms RK4 step, the cascaded PID controller with the gains in `core/controller.h`, flying on the RTK estimate (ZED-F9P GPS, BMI088 IMU, IST8310 magnetometer, gyro calibrated for 1 s first), the VLP-16 lidar at 10 Hz with replanning when new cells cut the path, 5 m/s cruise and 3 m/s² ramps
- winds: every trip flown twice with the same sensor noise (`seed + 30000`), once in calm air and once in a 4 m/s breeze from the west plus 1.5 m/s gusts (gauss markov, tau 2 s, the middle of the `results/wind.md` grid)
- a flight ends 3 s after the setpoint reaches the end of the path, or on a crash (`crashed()`: the drone's center within its 0.356 m reach of a building or the ground), or at 120 s
- success: no crash, inside 120 s, and the real drone within 1 m of the goal at the end. A trip with no plan would count as a failure in both winds (there were none)
- tracking error: rms over the flight of the real drone's distance from the setpoint. Estimation error: rms of the estimate's distance from the real drone
- whole run: 265.1 s for 2000 flights

## Results

| | calm | 4 m/s breeze + 1.5 m/s gusts |
|---|---|---|
| success | 1000 of 1000 (100%) | 999 of 1000 (99.9%) |
| crashed | 0 | 1 (seed 962) |
| out of time / ended over 1 m away | 0 / 0 | 0 / 0 |
| tracking error, rms per flight (median, p95, worst) | 0.109, 0.159, 0.472 m | 0.144, 0.190, 0.497 m |
| estimation error, rms per flight (median, p95, worst) | 0.013, 0.015, 0.022 m | 0.013, 0.015, 0.022 m |
| off the goal at the end (median, p95, worst) | 0.081, 0.117, 0.292 m | 0.098, 0.182, 0.912 m |
| flight time (median, p95, worst) | 13.95, 20.13, 30.56 s | 13.90, 20.13, 30.56 s |
| flights with a replan | 19 replans over the 1000 flights, at most 1 per flight | 22, at most 1 per flight |
| replan time, a* + smooth (median, p95, worst) | 1.33, 4.58, 4.58 ms | 1.55, 16.36, 20.82 ms |
| a replan found no path | 8 | 6 |

Planning, the same for both winds:

| | median | p95 | worst |
|---|---|---|---|
| a* + smooth (ms) | 10.91 | 48.99 | 140.13 |
| path length / straight line | 1.009 | 1.091 | 1.184 |
| straight line distance (m) | 36.4 | 59.1 | 77.2 |

A path was found for all 1000 trips.

## The crash

Seed 962 in the wind. The goal (-2.25, -27.75, 2.25) is in a free cell right next to a building, so the last 1 m leg runs along its margin. The planner keeps the drone's center 0.79 m from buildings and the drone reaches 0.356 m, which leaves 0.43 m of slack. At 13.3 s the drone was 0.80 m from its setpoint (pushed east and down by the breeze and a gust, at 1.87 m/s) and its reach touched the wall. The same trip in calm air succeeded. So in wind the margin is the limit, not the controller or the estimate (the estimate was within a centimeter of the real drone there). A bigger margin when it's windy, or slowing down on the last leg next to a wall, would be the fix. I haven't changed it, since that would change every planning number before it.

## The replans that found no path

8 calm and 6 windy flights had a replan that couldn't find a way and held where it was, but every one of them still ended within 1 m of the goal, so they count as successes.
