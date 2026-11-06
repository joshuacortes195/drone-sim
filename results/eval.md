# Batch evaluation (phase 7)

Measured 2026-11-04 on my Intel Mac with `tools/batch_eval.cpp`, then rerun 2026-11-05 on the same Mac after the mixer change that fixed the one crash (see "The crash"). The results table is the rerun. The headline run is meant for my PC like phases 4 to 6a, so this table gets rerun there and both kept, labeled.

Rerun 2026-11-06 on the same Mac after the heading cap (`results/search.md`, "The crash this found"): the results table was that run until the rerun below (its numbers are the ones in brackets now) and the 2026-11-05 numbers are in the table under it. Still 1000 of 1000 in both winds. Most numbers moved a little, the windy worst cases got a bit worse (tracking 0.469 to 0.530 m, off the goal 0.383 to 0.426 m), and the same number of replans found no path (4 and 7).

Rerun again on the same Mac after braking replans (2026-11-06, commits 6f65b20 and d1c7dcb, the second one the review fixes). A replan at speed used to start the new path from rest, so the setpoint stopped dead and the drone overshot it. Now `replan()` plans from where the drone would stop braking straight ahead at 5 m/s², and the setpoint brakes into the new path from the drone's current velocity. If that braking line would come within the planner's margin of something known (the same 0.79 m every planned leg keeps), or there's no path from the stopping point, it falls back to the old way. The results table is now that run, with the heading cap numbers in brackets where they moved. Almost nothing did. Success, crashes, tracking, estimation, off the goal, the replan counts and the failed replans came out the same to the last digit. What moved: the calm median flight time (13.938 to 13.951 s), the calm worst peak tilt (35.500 to 35.545°, now seed 396 instead of 468) and the windy p95 peak tilt (33.88 to 33.94°). The whole run has only 21 replans calm and 17 windy across its 2000 flights, so there are few places braking could change anything here. The two worst tracking flights didn't move (see seeds 630 and 820 below). A first braking run before the review fixes (a looser check on the braking line) moved a few more rows, a bit worse in places: windy peak tilt worst 35.80° and 18 windy replans.

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
- tracking error: rms of the real drone's distance from the setpoint over every step flown, including the 3 s hold at the goal (about a fifth of a median flight), which is why it comes out below the 0.114 m in `results/control.md` for flying a route. Estimation error: rms of the estimate's distance from the real drone
- crashed flights are left out of the error and time stats, so a crash doesn't read as a tracking or goal error
- whole run: 245.6 s for 2000 flights on an idle machine (2026-11-04). The 2026-11-05 rerun shared the machine with a compile and two other runs and took 1482 s, so its times aren't used: the planning times below are from 2026-11-04 (same planner, same trips, same paths), and the replans are only counted. The 2026-11-06 rerun took 283.7 s but shared the machine with a browser rendering the sim, so its times aren't used either. Its path length ratios and straight line distances came out identical, so the plans are the same. A second 2026-11-06 run on an idle machine (258.5 s, the dev server open but no browser) printed the same flight numbers to the last digit, plus the new peak tilt and worst flight lines, and its planning times are the ones in the planning table now. The braking replans rerun (commit d1c7dcb) took 270.6 s with the dev server open and nothing else running. Braking doesn't touch the first plan, and its path length ratios and straight line distances came out identical, so the planning table stays on the idle 10.73 ms run: that run's 11.07 ms median (max 130.0 ms against 105.0) is run to run noise on a laptop. Run with `./build/batch_eval 1000`, or `./build/batch_eval 1 962` to fly one seed's trip on its own

## Results

| | calm | 4 m/s breeze + 1.5 m/s gusts |
|---|---|---|
| success | 1000 of 1000 (100%) | 1000 of 1000 (100%) |
| crashed | 0 | 0 |
| out of time / ended over 1 m away | 0 / 0 | 0 / 0 |
| tracking error, rms per flight (median, p95, worst) | 0.103, 0.150, 0.657 m | 0.141, 0.184, 0.530 m |
| estimation error, rms per flight (median, p95, worst) | 0.013, 0.015, 0.020 m | 0.013, 0.015, 0.019 m |
| off the goal at the end (median, p95, worst) | 0.078, 0.111, 0.226 m | 0.095, 0.180, 0.426 m |
| flight time (median, p95, worst) | 13.95 (13.94), 20.11, 30.56 s | 13.90, 20.11, 30.56 s |
| replans | 21, at most 2 per flight | 17, at most 1 per flight |
| a replan found no path | 4 | 7 |
| peak tilt per flight (median, p95, worst) | 27.97°, 31.42°, 35.55° (35.50°) | 27.70°, 33.94° (33.88°), 35.07° |
| worst flights (seed): tracking, off the goal, tilt | 630, 432, 396 (468) | 820, 645, 468 |

Before the heading cap (2026-11-05, same trips). Success, crashes, out of time and the failed replan counts came out the same:

| | calm | 4 m/s breeze + 1.5 m/s gusts |
|---|---|---|
| tracking error, rms per flight (median, p95, worst) | 0.104, 0.152, 0.658 m | 0.142, 0.186, 0.469 m |
| estimation error, rms per flight (median, p95, worst) | 0.013, 0.015, 0.021 m | 0.013, 0.015, 0.021 m |
| off the goal at the end (median, p95, worst) | 0.078, 0.114, 0.254 m | 0.095, 0.181, 0.383 m |
| flight time (median, p95, worst) | 13.90, 20.11, 30.56 s | 13.90, 20.11, 30.56 s |
| replans | 22, at most 2 per flight | 16, at most 1 per flight |

Before the mixer change (2026-11-04, same trips):

| | calm | 4 m/s breeze + 1.5 m/s gusts |
|---|---|---|
| success | 1000 of 1000 (100%) | 999 of 1000 (99.9%) |
| crashed | 0 | 1 (seed 962) |
| tracking error, rms per flight (median, p95, worst) | 0.109, 0.159, 0.472 m | 0.145, 0.194, 0.610 m |
| estimation error, rms per flight (median, p95, worst) | 0.013, 0.015, 0.022 m | 0.013, 0.015, 0.020 m |
| off the goal at the end (median, p95, worst) | 0.081, 0.117, 0.292 m | 0.097, 0.188, 0.339 m |
| flight time (median, p95, worst) | 13.95, 20.13, 30.56 s | 13.94, 20.11, 30.56 s |
| replans | 19, at most 1 per flight | 23, at most 1 per flight |
| replan time, a* + smooth (median, p95, worst) | 1.26, 4.70, 4.70 ms | 1.51, 14.11, 21.94 ms |
| a replan found no path | 8 | 6 |

The windy column there leaves out the crash (999 flights).

The change made the median and p95 a little better everywhere and the windy worst case better (0.610 to 0.469 m). The calm worst case got worse (0.472 to 0.658 m). That's the same kind of flight as seed 127 in `results/planning.md`: in a hard turn there's no thrust level where all the roll and pitch fits, so the mixer gives up some thrust, the drone sinks a little, and the outer loop spends a moment climbing back with no thrust left for sideways force.

The heading cap barely moved the calm worst case (0.658 to 0.657 m) and made the windy one a bit worse (0.469 to 0.530 m). I traced both with a one seed scratch copy of `batch_eval` that logs the drone against its setpoint. Neither is the seed 127 kind of flight, so that explanation above doesn't hold for them. Both are a replan at cruise speed. On seed 630 (calm) the lidar sees something 8.1 s in while the drone is doing 4.8 m/s; on seed 820 (windy) it's 3.1 s in at 4.5 m/s. The new hits put the drone's own cell inside the new obstacle's margin, so `replan()` starts the new path from the nearest free cell A* found, 1.1 m to the side, and the new path starts from rest. The drone can't stop in no distance: it overshoots the restarted setpoint by up to 2.57 m (seed 630) and 2.12 m (seed 820), tilting to 35° to brake, and takes about 2 s to get back on it. The overshoot is along the old direction of travel, which is toward whatever the lidar just saw, so the fix I tried next was braking replans (commits 6f65b20 and d1c7dcb, see the top): the new path starts where the drone would stop and the setpoint brakes into it from the drone's velocity. It doesn't change these two at all (still 0.657 and 0.530 m). I traced them again on the first braking build: on both, the straight braking line runs into the new obstacle, so `replan()` falls back to the old start. The review fixes only made that check stricter (the braking line now has to keep the planner's full margin), and both flights came out the same to the digit, so they still fall back, but I haven't retraced them on the final build. What they need is to brake and swerve at once, and the old start from the nearest free cell to the side already does that. Both flights still reach the goal.

Planning, the same trips for both winds:

| | median | p95 | worst |
|---|---|---|---|
| a* + smooth (ms) | 10.73 (10.38 on 2026-11-04) | 48.13 (44.58) | 104.98 (114.78) |
| path length / straight line | 1.009 | 1.091 | 1.184 |
| straight line distance (m) | 36.4 | 59.1 | 77.2 |

A path was found for all 1000 trips. The planning times are wall clock on a laptop, so they move a little between runs: a first run of the same trips (before the fixes below) had 10.91, 48.99 and 140.13 ms. The paths themselves came out the same.

## The crash (fixed 2026-11-05)

Seed 962 in the wind. It crashed in both runs, with two different gust streams, so it isn't one unlucky gust. The path ends with a 1 m leg flown east, with the breeze behind it, to a goal (-2.25, -27.75, 2.25) right next to a building. A 1 m leg at 3 m/s² tops out at 1.7 m/s, but a gust (the air reached 5.5 m/s) pushed the drone to 2.3 m/s. Braking against the breeze then tilted it up to 53°. Hovering takes 660 of the motors' 838 rad/s, so with thrust going as speed squared they have 1.61 times hover thrust, which runs out of lift past about 52° of tilt (worked out from the parameters in `core/quad.h`, not measured). It sank 0.6 m, swung around the goal at up to 1.4 m/s, and its reach touched the building 0.91 m from the goal at 13.3 s, about a second after the setpoint had arrived. The estimate was within a centimeter of the real drone the whole time, and the same trip in calm air succeeded.

I first read that as running out of thrust margin. Tracing it showed something else. The controller never asked for more than 34° of tilt. The drone went to 53° on its own while the command was 25° and falling, and one rotor sat at full speed the whole time. The attitude loop wanted about 2.4 N m of torque to stop the tip, but the mixer kept the total thrust exact first and only fit roll and pitch on top, so it handed out 17 to 21% of what was asked. There was no headroom left on the rotor that needed to speed up.

The fix is in `mix()` in `core/controller.cpp`: roll and pitch now come before thrust. If moving the total thrust up or down makes all of the roll and pitch fit, it moves just that far. If no thrust level fits it all, it goes to the middle of the range, where opposite rotors have the same room and the most of it fits. Yaw still gives way first. Two new tests in `tests/test_controller.cpp` cover both ends (near full thrust and near zero), and the old "roll too big" test now checks that a rotor ends up at both limits, meaning no other thrust level would have fit more. Seed 962 then peaked at 35.5° against a command of 34.4° and ended 0.38 m from the goal. Since the heading cap its windy flight peaks at 31.3° over the whole trip (`./build/batch_eval 1 962`).

I also tried only moving thrust when that makes everything fit, and keeping it otherwise. That also stopped the crash, but seed 962 still tipped to 50°, and its 1000 trip run had a windy worst case of 0.724 m. The middle of the range fixes the tipping itself, so I kept that.

## The replans that found no path

4 calm (seeds 96, 432, 656, 842) and 7 windy (seeds 107, 359, 397, 432, 462, 527, 645) flights had a replan that couldn't find a way and held where it was (the same seeds, distances and times before braking replans, the same counts before the heading cap, 8 and 6 before the mixer change). All 11 were on the final approach or already there: 0.04 to 0.34 m from the goal (0.06 to 0.25 m before the heading cap) and at most 0.56 s (0.47 s) before the setpoint reached the end, so every one still ended within 1 m and counts as a success (the tool prints each one). Those goals sit next to a building, and a lidar hit late in the flight blocks the goal's cell, so there's nowhere left to plan to. In the page, a goal tapped right next to a wall used to turn into "the lidar found the way blocked" once the lidar looked at that wall. Since 2026-11-05 the page counts a failed replan within 1 m of the goal as arrived, the same 1 m rule as here.

## Fixes after a review pass

The first run counted the crash in the error stats (its 0.91 m at impact was the windy "worst off the goal"), seeded the gusts with the next trip's sensor seed, and counted one step too many in each rms. All three are fixed above. The success counts didn't change.
