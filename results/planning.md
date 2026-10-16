# Planning results (phase 4)

Measured 2026-10-16 on my PC with `tools/plan_report.cpp`, plus a browser check. This is the first results file measured on the PC. The earlier ones (physics, control, browser) are from my Intel Mac, so don't compare times across them.

## How

- machine: AMD Ryzen 7 3700X (8 cores, 16 threads), 16 GB RAM, Windows 10 Home 19045
- native build: MSVC 19.44 (Visual Studio 2022 Build Tools), CMake Release build (`/O2 /Ob2 /DNDEBUG`), C++17, double precision
- browser build: emcc 6.0.12, the same CMake Release settings, Chrome 155 (the build agent-browser installs), driven by Playwright 1.64
- maps: `city(seed)` for seeds 0 to 199. 5 x 5 lots of 8 m on a 12 m grid (4 m streets), home lot left empty, each other lot has an 80% chance of one building 4 to 8 m on a side and 4 to 18 m tall
- grid: 60 x 60 x 20 m at 0.5 m cells (120 x 120 x 40 = 576,000 cells). Buildings grown by 0.789 m (0.256 m arm + 0.1 m rotor + 0.433 m half cell diagonal), ground and ceiling kept the same 0.789 m away
- queries: one per seed, start and goal are random free cells at least 20 m apart, picked with `std::mt19937(seed + 1000)`
- planner: 26 neighbor A* with the straight line heuristic and no corner cutting, then line of sight smoothing against the grown boxes
- flying: the real phase 1 physics and phase 2 controller (same gains as `results/control.md`), 1 ms steps, flying on the true state. The setpoint slides along the smoothed path at 5 m/s cruise with 3 m/s² ramps and stops at every waypoint, with velocity and acceleration fed forward. A leg slows down if it would climb or sink faster than 1.5 m/s. Each flight runs 3 s past the end of the path
- each time is one run, timed with `std::chrono::steady_clock` natively and `performance.now()` in the browser
- run it: build, then `build/plan_report.exe`. `build/plan_report.exe queries` prints each query with its smoothed path's waypoint count and length

## Planning

200 of 200 queries found a path. 0 smoothed paths come closer to a building than the drone's 0.356 m reach (checked with `path_blocked` against the boxes grown by 0.356 m).

| native | median | 95th percentile | worst |
|---|---|---|---|
| build the grid | 0.61 ms | 0.78 ms | 0.89 ms |
| A* | 15.5 ms | 62.3 ms | 135.2 ms |
| smoothing | 0.002 ms | 0.009 ms | 0.016 ms |
| straight line distance | 37.4 m | 57.3 m | 78.5 m |
| A* path length / straight line | 1.100 | 1.164 | 1.249 |
| smoothed length / straight line | 1.005 | 1.086 | 1.183 |
| A* waypoints | 68 | 108 | 122 |
| smoothed waypoints | 3 | 5 | 7 |

Other runs gave the same lengths (they don't depend on timing) and A* medians from 14.9 to 15.6 ms with worst cases from 130 to 149 ms, so the times move by a few percent between runs.

What it means:

- the A* path on its own is about 10% longer than a straight line, mostly from the 45 degree staircase a grid forces. Smoothing takes that down to 0.5% at the median, so most of the extra length was the grid, not the buildings
- smoothing cuts about 68 grid waypoints down to 3, which matters because the drone stops at every waypoint
- A* is the only part that costs anything, and its worst case is about 9 times its median. I haven't measured which queries are the slow ones yet

## Same plans in the browser

The same 200 queries, planned by the wasm build in Chrome (`fly_to()`, which is A* plus smoothing), in two runs:

| | median | 95th percentile | worst |
|---|---|---|---|
| plan in wasm, run 1 | 7.6 ms | 31.9 ms | 73.6 ms |
| plan in wasm, run 2 | 7.3 ms | 32.6 ms | 74.3 ms |

- all 200 paths have the same number of waypoints as the native build and the same length to the last bit (largest difference 0 m), so the seeded city and the planner give identical answers in both builds
- the wasm build plans about twice as fast as my MSVC build on the same machine. I haven't looked into why, different compilers (clang vs MSVC) and standard libraries are the obvious difference
- a plan runs inside one physics step on the page, so the worst one here holds up the frame it lands in by about 74 ms

## Flying the plans

| native, 200 flights | median | 95th percentile | worst |
|---|---|---|---|
| RMS distance to the setpoint | 0.102 m | 0.152 m | 0.178 m |
| worst distance to the setpoint | 0.283 m | 0.401 m | 0.430 m |
| closest the drone's center got to a building or the ground | 1.151 m | 4.661 m | 9.616 m |
| distance from the goal 3 s after the path ends | 0.086 m | 0.100 m | 0.112 m |

The closest any flight got was 0.680 m from the drone's center to a building or the ground. The drone reaches 0.356 m from its center, so the tightest pass still left 0.32 m of air. 0 flights touched anything.

Each flight starts hovering, already settled, on the first cell center. In the page the drone starts a plan from wherever it is, which can be up to 0.43 m from that cell center, so the first moments of a real flight aren't covered here.

The column that matters for safety is the smallest one, which is why the 0.680 m is listed on its own. The "worst" column there is the flight that stayed farthest away.

### Before the climb limit

The first version slid the setpoint along every leg at 5 m/s no matter how steep it was. Flown the same way:

| native, 200 flights | median | 95th percentile | worst |
|---|---|---|---|
| RMS distance to the setpoint | 0.114 m | 0.517 m | 4.352 m |
| worst distance to the setpoint | 0.312 m | 1.361 m | 7.340 m |

Every flight that fell more than 1.3 m behind had a steep leg. The worst (seed 163) drops 17 m over a 20.3 m leg, which at 5 m/s asks for 4.2 m/s down, and the controller caps vertical speed at 2 m/s. Slowing steep legs so they never ask for more than 1.5 m/s up or down (3/4 of the cap, to leave the feedback room) brought the worst case from 7.34 m to 0.43 m.

## In the page

The page flies seed 1 through five street crossings at 3 to 12 m up, planning each leg with `fly_to()`, and draws the planned path.

- no console errors or page errors over 62 s, in both a headless and a normal Chrome window
- 1000 physics steps per second (999 to 1001 in every half second window)
- 180 fps in a normal Chrome window. I think that's my monitor's refresh rate but I haven't checked its setting
- the readout sampled every 2.5 s showed the drone at about the same height at the same times in both runs (9.9 m at 62 s in both, 7.3 m and 7.2 m at 59.5 s). The samples don't land on the same physics step, so this only shows the flights are close, not identical
- wasm file 38.3 kB (19.3 kB gzipped), up from 19.9 kB with the planner and city added
- wasm memory raised from 16 MB to 64 MB so A* has room on the full grid
