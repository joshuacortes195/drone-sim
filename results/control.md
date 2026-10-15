# Control results (phase 2)

Measured 2026-10-15 with `tools/fly_route.cpp` unless a section says otherwise. Everything except the two modified-copy windup numbers is printed by that tool.

## How

- machine: Intel Core i5-8259U @ 2.30GHz, 8 GB RAM, macOS 15.7.7
- compiler: Apple clang 17.0.0, CMake Release build (`-O3 -DNDEBUG`), C++17, double precision
- airframe: RotorS Iris parameters from `docs/plan.md`, flown with the phase 1 physics (RK4, motor lag, rotor drag)
- timestep: 1 ms physics. Rate and attitude loops every step (1 kHz), position and velocity every 20 steps (50 Hz)
- the controller flies on the true state (no sensor noise yet, that's phase 5a)
- every run starts hovering at 10 m with the rotors already at hover speed
- run it: `cmake --build build && ./build/fly_route`

## Gains

All in `Gains` in `core/controller.h`.

| loop | x / roll | y / pitch | z / yaw | units |
|---|---|---|---|---|
| position P | 1 | 1 | 1.5 | (m/s) per m |
| velocity P | 2.5 | 2.5 | 4 | (m/s²) per (m/s) |
| velocity I | 1 | 1 | 1 | (m/s²) per m |
| attitude P | 7 | 7 | 3 | (rad/s) per rad |
| rate P | 20 | 20 | 8 | (rad/s²) per (rad/s), times inertia for torque |
| rate I | 10 | 10 | 2 | (rad/s²) per rad |
| max rate | 4 | 4 | 0.8 | rad/s |

Limits: velocity integral capped at 2 m/s² per axis, rate integral at 1 rad/s² per axis, speed 8 m/s horizontal and 2 m/s vertical, tilt 35°, collective thrust between 0.2 and 0.9 of full (full is 4 × 6.003 N = 24.01 N). No D terms: nothing in the tests needed one.

## Step responses

1 m set point steps in x, y and z, and a 1 rad heading step, 10 s each. Overshoot is past the target as a share of the step. Settle is the last time the response was outside a 2% band. The altitude dip is the most the height moved while a different axis was stepping.

| step | overshoot | settle (2%) | error at 10 s | altitude dip | test limit |
|---|---|---|---|---|---|
| x 1 m | 1.1% | 1.85 s | 1.7 mm | 14.0 mm | < 10%, < 4 s |
| y 1 m | 1.0% | 1.86 s | 1.7 mm | 8.3 mm | < 10%, < 4 s |
| z 1 m | 1.6% | 1.41 s | 1.0 mm | | < 10%, < 3 s |
| yaw 1 rad | 2.2% | 2.00 s | 0.0009 rad | 8.0 mm | < 10%, < 4 s |

The test limits (also under 1 cm or 0.01 rad at the end, under 5 cm of altitude dip) are in `tests/test_controller.cpp`.

## Route tracking

A fixed route of 5 waypoints, 81.2 m long: (0, 0, 10) → (20, 0, 10) → (20, 20, 15) → (0, 20, 15) → (0, 0, 10). A reference point moves along it and the drone chases it, with the point's velocity and acceleration fed forward. Error is measured every 1 ms while the reference is moving: the distance to the reference point, and the cross track error (distance to the nearest part of the route line).

| reference | cruise speed | time | RMS to reference | max to reference | RMS cross track |
|---|---|---|---|---|---|
| 3 m/s² ramps, stops at each waypoint | 2 m/s | 43.3 s | 0.061 m | 0.209 m | 0.038 m |
| 3 m/s² ramps, stops at each waypoint | 5 m/s | 22.9 s | 0.119 m | 0.219 m | 0.062 m |
| constant speed, instant 90° corners | 2 m/s | 40.6 s | 0.298 m | 1.147 m | 0.202 m |
| constant speed, instant 90° corners | 5 m/s | 16.2 s | 2.904 m | 5.506 m | 1.892 m |

The sharp corner rows are a reference the drone physically can't follow. Turning 90° at 5 m/s is a 7.1 m/s change in velocity, the 35° tilt limit gives at most 6.9 m/s² sideways, so it spends over a second pinned at the tilt limit while the error grows to about 5 m. Those rows show what the planner's path smoothing (phase 4) needs to avoid, not how well the controller tracks.

## Feed forward

Same ramped route, before and after adding the path's acceleration to the velocity loop's output (`sp.acc` in `core/controller.cpp`), gains unchanged:

| cruise speed | RMS to reference, velocity feed forward only | with acceleration feed forward too |
|---|---|---|
| 2 m/s | 0.220 m (max 0.657 m) | 0.061 m (max 0.209 m) |
| 5 m/s | 0.652 m (max 1.051 m) | 0.119 m (max 0.219 m) |

Without it, the velocity loop has to fall behind by a / kv = 3 / 2.5 = 1.2 m/s before it asks for the 3 m/s² the ramp needs.

## Recovering from a tipped start

Hovering at 10 m with the set point right there, but tipped about x by the angle shown, then 10 s of flight.

| start tilt | worst distance from set point | height lost | back under 5° tilt | off at 6 s | off at 10 s |
|---|---|---|---|---|---|
| 30° | 0.222 m | 0.039 m | 0.18 s | 1.71 cm | 0.44 cm |
| 120° | 4.100 m | 2.822 m | 0.61 s | 11.2 cm | 3.7 cm |
| 180° (upside down) | 9.892 m | 8.970 m | 0.87 s | 1.106 m | 2.3 cm |

The slow last few centimeters come from the velocity integral, which charged up during the first lurch and bleeds off with a time constant of kv / ki = 2.5 s. From upside down it loses almost 9 m: the thrust never goes below its 20% floor, and while the drone is still inverted that floor pushes it down. The floor has to be there anyway, because with zero thrust the mixer has nothing to steer with. Before I added it, a drone started upside down never turned over and fell 481 m in 10 s.

## 10 m step in x

A plain set point jump with no path, so the speed cap, the tilt limit and the integral all come into play: 9.937 m at 3 s, 9.801 m at 5 s (it brakes a little early and creeps back in), 9.9955 m at 15 s, and no overshoot past 10 m.

## Windup

From `tests/test_controller.cpp`: held still for 5 s with the goal 10 m away, then let go, its path matches a run that was never held to under 0.1 mm at every 100 ms sample. Measured once with a modified copy, everything else the same: with the rate integral cap at 5 instead of 1 the two paths differ by up to 7.6 cm, and with no rate cap at all they're up to 19.8 m apart.

## Cost

One `control()` call (the outer loop runs on every 20th call, the mixer's 4×4 solves on every call), averaged over 2,000,000 calls: 268 to 300 ns across six runs.

## Tuning history

What changed while tuning, all measured with the same step scoring:

- first gains: yaw overshot 31.3% (yaw rate cap 1.5 rad/s, yaw rate I 4, rate integral cap 5). With the cap at its final value of 1 those yaw gains still overshoot 18.8%. Yaw torque can only give about 1.5 rad/s², so from 1.5 rad/s it needs about 0.75 rad to stop, but the attitude P only starts slowing down 0.5 rad out. Capping yaw rate at 0.8 rad/s and setting yaw rate I to 2 brought it to 2.2%
- velocity I 0.5 → 1: x settle went from 2.05 s to 1.85 s with overshoot 0.9% → 1.1%, and the 10 m move ended closer
- tried conditional integration (stop adding to the velocity integral while the tilt or thrust limit clips the output). On the 10 m move (speed cap still 5 m/s then) it was at 9.54 m after 3 s, against 9.90 m without it, because it also stopped the integral learning the rotor drag push it needs at cruise speed. Removed it. The caps do the anti windup work
- rate integral cap 5 → 1 rad/s²: rotor drag at 5 m/s makes about 0.53 rad/s² of pitch (4 rotors × 8.06e-5 × 660 rad/s × 5 m/s ≈ 1.06 N, acting 0.023 m above the center, over Iyy 0.0459), and the rolling moment adds about 0.38 rad/s² of roll (4 × 1e-6 × 660 × 5 = 0.0132 N m over Ixx 0.0348). Both fit under 1, which winds up far less
- horizontal speed cap 5 → 8 m/s: with a 5 m/s path and a 5 m/s cap, the velocity set point had no room to catch up once it fell behind
