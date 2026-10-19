# Lidar and replanning results (phase 5b, core)

Measured 2026-10-18 on my PC with `tools/lidar_report.cpp` and the lidar tests.

## How

- machine: AMD Ryzen 7 3700X (8 cores, 16 threads), 16 GB RAM, Windows 10 Home 19045
- build: MSVC 19.44 (Visual Studio 2022 Build Tools), CMake Release build (`/O2 /Ob2 /DNDEBUG`), C++17, double precision
- trips: the same 200 seeded cities, starts and goals as `results/planning.md` (seeds 0 to 199). The drone plans on the city it knows, then a 2 m cube it doesn't know about is dropped floating on the middle of the path's longest leg. Every trip's longest leg was over 6 m, so the cube never covers either end
- each trip is flown twice with the same sensor noise (seed = map seed + 5000): once blind, once with the lidar
- flying: on the RTK estimate (ZED-F9P GPS, BMI088 IMU, IST8310 magnetometer, same as `results/estimation.md`), 1 s gyro calibration on the pad, 5 m/s cruise, 3 m/s² ramps, 1 ms physics
- lidar: Velodyne VLP-16, one full turn every 100 steps (10 Hz), 50 steps after each GPS fix. A turn is taken all at once from the true pose, and the hits are placed with the estimated pose. New hits go into the grid and the box list. If the boxes from this turn cut the path being flown, it plans again from the estimate (A* plus smoothing) and starts the new path from rest
- closest distances are from the drone's true center to the outside of the boxes. The drone's reach is 0.356 m
- timings are wall clock around `scan()` + `add_hits()` and around `astar()` + `smooth()`
- run it: build, then `build/lidar_report.exe` (about 30 s)

## The lidar

| | value | source |
|---|---|---|
| beams | 16, from -15° to +15°, 2° apart | Velodyne Puck (VLP-16) data sheet 63-9229 Rev-K |
| around | 360° | same |
| steps around | 2° (180 per turn) | my choice. At 10 Hz the real one steps 0.2°. I cut it 10x so a turn is 2,880 rays |
| range | 100 m | data sheet |
| spin rate | 10 Hz | data sheet allows 5 to 20 Hz |
| range noise | σ = 3 cm | data sheet says "up to ±3 cm (typical)" with no σ, so I treat 3 cm as σ |
| weight | about 830 g | data sheet. More than an Iris can really carry. I don't add it to the mass |

Only boxes reflect. The ground and ceiling are known and never scanned.

## Results

| | blind | with lidar |
|---|---|---|
| flights with the center inside something | 200 (all inside the cube) | 0 |
| flights within the drone's reach of something | 200 | 1 |
| closest to anything, median / p95 / worst over flights | 0 / 0 / 0 m | 1.140 / 1.694 / 1.795 m (worst flight 0.112 m) |
| ended within 1 m of the goal | | 200 |
| off the goal at the end, median / p95 / max | | 0.070 / 0.110 / 0.153 m |

With the lidar on:

| | median | p95 | max | n |
|---|---|---|---|---|
| distance to the cube at the first replan (m) | 15.0 | 27.1 | 37.7 | 200 |
| replans per flight | 2 | 5 | 7 | 200 |
| `scan()` + `add_hits()` per turn (ms) | 0.726 | 1.120 | 2.004 | 33,655 |
| replan, `astar()` + `smooth()` (ms) | 10.4 | 43.1 | 136.7 | 543 |

Every flight replanned at least once. One replan found no path (seed 47, the drone held for a turn, then a later replan got through), and that flight still reached its goal.

The one close call is seed 163: 0.112 m from the cube, center never inside. Its longest leg dives 56.7° from level, from 18.25 m down to 1.25 m. The VLP-16 only looks 15° up and down, so a cube on a steep dive stays under every beam until the drone is almost on it. Its first replan came 0.68 m from the cube at 1.79 m/s. That's a limit of this sensor on steep legs, not a bug, and it's still open.

## What changed while measuring

Each row is one full run of the 200 trips with the lidar on.

| version | inside something | within reach | reached goal | replan found no path | closest |
|---|---|---|---|---|---|
| a hit is skipped if its cell is already blocked | 2 | 14 | 200 | 0 | 0.000 m |
| one box per cell with a hit, known buildings matched within 0.1 m | 0 | 1 | 196 | 20 | 0.112 m |
| known buildings matched within 0.1 m + 0.03 per meter of range (kept) | 0 | 1 | 200 | 1 | 0.112 m |

- First version: all 14 close calls were the cube. A hit was skipped when its cell was already blocked, but a cell is often blocked only because it's inside the margin of a neighboring hit. Those skipped hits never became boxes, and `smooth()` and the replan check only look at the box list. So the cube's sides, seen after its front, never cut the path. Seed 127 replanned once on the pad, then flew within 1 cm of the cube without replanning again. A test now scans a cube from the front, then from its side, and checks that every spot near the side is inside a box
- Second version: all 20 failed replans had the goal cell blocked by fake obstacles, up to 2,015 hit boxes on one flight when a 2 m cube makes about a hundred. They were hits on known buildings placed more than 0.1 m off the wall by the estimate's attitude error. I measured that error at every lidar turn of that run (about 30,000 turns, with a temporary build): median 4.6 mrad, p95 13.0 mrad, p99 17.6 mrad, worst 26.6 mrad. At 30 m the worst case moves a hit 0.8 m. So the known building tolerance now grows with range, with 0.03 rad picked just over that worst case

## Limits

- the whole turn is taken in one instant. A real VLP-16 sweeps over 0.1 s while the drone moves up to 0.5 m
- the lidar only adds obstacles, it never clears them. Nothing here moves, so that's fine for now
- the planner treats space it hasn't seen as free. Going around a box's front corner, the new path can cut behind it, and the side only shows up once the drone gets past the front. That's why most flights replan 2 to 5 times
- each replan starts the new path from rest, so the drone brakes hard even when the new path heads the same way
- the worst replan (137 ms) runs inside one physics step natively. In the page it would hold up a frame
