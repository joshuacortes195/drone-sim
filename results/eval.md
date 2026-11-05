# Batch evaluation (phase 7)

Measured 2026-11-04 on my Intel Mac with `tools/batch_eval.cpp`. The headline run is meant for my PC like phases 4 to 6a, so this table gets rerun there and both kept, labeled.

## How

- machine: Intel Mac, Core i5-8259U @ 2.30GHz (4 cores, 8 threads), 8 GB RAM, macOS 15.7.7
- build: Apple clang 17.0.0, CMake Release build (`-O3 -DNDEBUG`), C++17, double precision, one thread
- maps: seeds 0 to 999, one seeded city each from `city()` in `core/map.cpp` (5 x 5 lots, 8 m lots on a 12 m grid, buildings 4 to 18 m tall, 20% of lots empty)
- trips: one per city. Start and goal are random free cells of the planning grid (0.5 m voxels, buildings grown by 0.79 m) at least 20 m apart, from their own random stream (`seed + 20000`), so they aren't the trips in `results/planning.md` or `results/lidar.md`
- planning: 26 neighbor A* plus line of sight smoothing, timed once per trip
- flying: the full stack, the same as the page. Physics on a fixed 1 ms RK4 step, the cascaded PID controller with the gains in `core/controller.h`, flying on the RTK estimate (ZED-F9P GPS, BMI088 IMU, IST8310 magnetometer, gyro calibrated for 1 s first), the VLP-16 lidar at 10 Hz with replanning when new cells cut the path, 5 m/s cruise and 3 m/s² ramps
- winds: every trip flown twice, once in calm air and once in a 4 m/s breeze from the west plus 1.5 m/s gusts (gauss markov, tau 2 s, the middle of the `results/wind.md` grid, gust seed `seed + 10000`). Both flights start from the same sensor seed (`seed + 30000`), so they have the same starting biases. The noise doesn't stay in step after that: the lidar takes one noise draw per hit, and a drone leaning into the wind hits a different number of things
- a flight ends 3 s after the setpoint reaches the end of the path, or on a crash (`crashed()`: the drone's center within its 0.356 m reach of a building or the ground), or at 120 s
- success: no crash, inside 120 s, and the real drone within 1 m of the goal at the end. A trip with no plan would count as a failure in both winds (there were none)
- tracking error: rms of the real drone's distance from the setpoint over every step flown, including the 3 s hold at the goal (about a fifth of a median flight), which is why it comes out below the 0.119 m in `results/control.md` for flying a route. Estimation error: rms of the estimate's distance from the real drone
- crashed flights are left out of the error and time stats, so a crash doesn't read as a tracking or goal error
- whole run: 245.6 s for 2000 flights. Run with `./build/batch_eval 1000`

## Results

| | calm | 4 m/s breeze + 1.5 m/s gusts |
|---|---|---|
| success | 1000 of 1000 (100%) | 999 of 1000 (99.9%) |
| crashed | 0 | 1 (seed 962) |
| out of time / ended over 1 m away | 0 / 0 | 0 / 0 |
| tracking error, rms per flight (median, p95, worst) | 0.109, 0.159, 0.472 m | 0.145, 0.194, 0.610 m |
| estimation error, rms per flight (median, p95, worst) | 0.013, 0.015, 0.022 m | 0.013, 0.015, 0.020 m |
| off the goal at the end (median, p95, worst) | 0.081, 0.117, 0.292 m | 0.097, 0.188, 0.339 m |
| flight time (median, p95, worst) | 13.95, 20.13, 30.56 s | 13.94, 20.11, 30.56 s |
| replans | 19, at most 1 per flight | 23, at most 1 per flight |
| replan time, a* + smooth (median, p95, worst) | 1.26, 4.70, 4.70 ms | 1.51, 14.11, 21.94 ms |
| a replan found no path | 8 | 6 |

The windy column leaves out the crash (999 flights).

Planning, the same trips for both winds:

| | median | p95 | worst |
|---|---|---|---|
| a* + smooth (ms) | 10.38 | 44.58 | 114.78 |
| path length / straight line | 1.009 | 1.091 | 1.184 |
| straight line distance (m) | 36.4 | 59.1 | 77.2 |

A path was found for all 1000 trips. The planning times are wall clock on a laptop, so they move a little between runs: a first run of the same trips (before the fixes below) had 10.91, 48.99 and 140.13 ms. The paths themselves came out the same.

## The crash

Seed 962 in the wind. It crashed in both runs, with two different gust streams, so it isn't one unlucky gust. The path ends with a 1 m leg flown east, with the breeze behind it, to a goal (-2.25, -27.75, 2.25) right next to a building. A 1 m leg at 3 m/s² tops out at 1.7 m/s, but a gust (the air reached 5.5 m/s) pushed the drone to 2.3 m/s. Braking against the breeze then tilted it up to 53°. Hovering takes 660 of the motors' 838 rad/s, so with thrust going as speed squared they have 1.61 times hover thrust, which runs out of lift past about 52° of tilt (worked out from the parameters in `core/quad.h`, not measured). It sank 0.6 m, swung around the goal at up to 1.4 m/s, and its reach touched the building 0.91 m from the goal at 13.3 s, about a second after the setpoint had arrived. The estimate was within a centimeter of the real drone the whole time, and the same trip in calm air succeeded.

So the limit here is thrust margin next to a wall, not the planner or the estimate. The planner keeps the center 0.79 m from buildings and the drone reaches 0.356 m, which leaves 0.43 m of slack, and this swing used more than that. Slowing the ramps in wind, a bigger margin when it's windy, or a tilt limit that gives up position before altitude would each help. I haven't changed any of them, since that would change every flight number before it.

## The replans that found no path

8 calm and 6 windy flights had a replan that couldn't find a way and held where it was. All 14 were on the final approach or already there: 0.03 to 0.26 m from the goal and at most 0.47 s before the setpoint reached the end, so every one still ended within 1 m and counts as a success (the tool prints each one). Those goals sit next to a building, and a lidar hit late in the flight blocks the goal's cell, so there's nowhere left to plan to. In the page that means a goal tapped right next to a wall can turn into "can't find a way" once the lidar looks at that wall.

## Fixes after a review pass

The first run counted the crash in the error stats (its 0.91 m at impact was the windy "worst off the goal"), seeded the gusts with the next trip's sensor seed, and counted one step too many in each rms. All three are fixed above. The success counts didn't change.
