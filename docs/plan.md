# Plan

An autonomous quadcopter simulator that runs in the browser. The drone has real flight physics, plans its own path through a city of boxes, follows it with a cascaded PID flight controller, sees obstacles with a simulated lidar and estimates where it is from noisy sensors. Visitors can set a goal, drop obstacles, add wind and watch it replan.

What matters most: tested controls, measured numbers and a clean live demo. Features come after that.

## Scope

In:

- one quadcopter, 6 degrees of freedom, quaternion orientation, four motors with lag, gravity and rotor drag
- cascaded PID: body rate, attitude, velocity, position
- 3D voxel A* with path smoothing and replanning
- city block maps of axis aligned boxes, generated from a seed
- the drone knows the seeded map from the start. Boxes a visitor drops are unknown to it until the lidar sees them, then it replans
- IMU (accelerometer and gyro with noise and bias), GPS with noise, a magnetometer, lidar raycasts
- linear Kalman filter on position, velocity and accelerometer bias, complementary filter for attitude. The controller flies on the estimate. A truth mode flies on the real state for comparison
- wind as a steady breeze plus gusts
- native batch evaluator over many seeded maps
- desktop first web app. On a phone it loads, flies the default mission and takes a tapped goal

Out, unless I decide otherwise later:

- RRT star (only if I ask for it)
- full error state EKF with attitude and bias states
- other obstacle shapes, moving obstacles, multiple drones
- battery model, ground effect, blade flapping, aerodynamics past the rotor drag and rolling moment above
- collisions that bounce. Touching a box ends the run as a crash

## The airframe

One coherent parameter set from a single published source, so I can defend every number:

**Source:** the Iris model in RotorS, ETH Zurich ASL. Furrer, Burri, Achtelik, Siegwart, "RotorS: A Modular Gazebo MAV Simulator Framework", in *Robot Operating System (ROS): The Complete Reference (Volume 1)*, Springer, 2016. Values read from `rotors_description/urdf/iris.xacro` in https://github.com/ethz-asl/rotors_simulator (master, checked 2026-10-14). The 3DR Iris is a 1.5 kg quad in the same class as an F450.

| Parameter | Value | Note |
|---|---|---|
| body mass | 1.5 kg | plus 4 rotors at 0.005 kg, 1.52 kg total |
| rotor positions (x forward, y left) | front x = 0.13, y = ±0.22 m. back x = -0.13, y = ±0.20 m | 0.023 m above the center. An X that's narrower front to back than side to side |
| inertia Ixx, Iyy, Izz | 0.0347563, 0.0458929, 0.0977 kg m² | diagonal |
| thrust constant k_f | 8.54858e-06 N/(rad/s)² | thrust = k_f ω² |
| moment constant | 0.016 m | rotor drag torque = 0.016 × that rotor's thrust |
| max rotor speed | 838 rad/s | |
| motor time constant | 0.0125 s up, 0.025 s down | first order lag |
| rotor radius | 0.1 m | |
| rotor drag coefficient | 8.06428e-05 | per rotor force = -k \|ω\| v⊥, where v⊥ is that rotor's air relative velocity with the part along the rotor axis removed. Applied at the rotor, so it also makes a torque |
| rolling moment coefficient | 1e-06 | per rotor torque = -k \|ω\| v⊥ |

The two drag terms are how RotorS does it (`rotors_gazebo_plugins/src/gazebo_motor_model.cpp`, which cites Martin and Salaün, "The True Role of Accelerometer Feedback in Quadrotor Control", ICRA 2010). Wind enters only through them, since v⊥ is measured against the air.

Spin directions: front right and back left spin counterclockwise, front left and back right spin clockwise.

Worked out from those values (checked again by a test in phase 1): hover rotor speed ω = sqrt(m g / (4 k_f)) ≈ 660 rad/s at 1.52 kg, thrust to weight at max speed ≈ 1.6. That's a modest margin, so the controller has to be smart about saturation.

The PX4 SITL Iris model (`PX4-SITL_gazebo-classic/models/iris/iris.sdf.jinja`) disagrees with RotorS on inertia, k_f and the moment constant. I don't mix the two.

## Conventions

- world frame: x east, y north, z up (ENU), meters, seconds, radians
- body frame: x forward, y left, z up (FLU), matches the RotorS model
- quaternion: Hamilton, stored w, x, y, z, rotates body vectors into the world
- gravity 9.81 m/s² along -z
- the core uses double precision. There's only one drone, so speed isn't a reason to use floats

## Timing

- physics: fixed 1 ms step. RK4 on the rigid body plus motor lag, quaternion renormalized after each step. The phase 1 torque free spin test decides whether RK4 is needed or semi implicit Euler is enough (free fall can't decide it, any second order method is exact under constant acceleration)
- controller: rate and attitude loops every physics step, position and velocity slower (start at 50 Hz, tuned in phase 2)
- sensors: IMU every physics step, GPS at 10 Hz, lidar at 10 Hz (settled in phase 5)
- browser: physics steps on a fixed clock with an accumulator and a per frame budget, rendering interpolates between states. Same pattern as the gravity sandbox

## Milestones

Each phase is one session, ends with tests passing, a commit, a reviewer pass, a robotics notes update and a breakdown update.

### Phase 0: plan (done 2026-10-14)

Scope questions, this plan, the rules file outside the repo, a CMake + doctest skeleton.

### Phase 1: flight physics (tdd, done 2026-10-14)

Concepts to learn first: rigid body dynamics, quaternions, thrust and torque from four motors.

- `core/vecmath.h`: small vec3 and quaternion helpers, only what the physics uses (the diagonal inertia meant no 3x3 matrix was needed)
- `core/quad.{h,cpp}`: parameters, state (position, velocity, quaternion, body rates, rotor speeds), `wrench()` (the forward mixer: rotor speeds to body force and torque, plus rotor drag), derivatives, `step(dt)`
- tests:
  - equal rotor speeds whose thrust matches the weight hold a hover (no drift, no rotation over 10 s). The arms are uneven but symmetric left to right and front to back in x, so equal thrusts give zero torque
  - spinning the clockwise pair (front left, back right) faster and the counterclockwise pair slower, with the same total ω², gives a positive yaw torque (nose turns left) of 4 × 0.016 × ΔT. Because the front arms are 0.22 m out and the back ones 0.20 m, it also gives a roll torque of 0.04 × ΔT, so the test checks that exact value instead of zero, and checks the yaw rate is positive after a short time
  - motors off from ω = 0 and at rest, it falls as z0 - g t² / 2 (rotor drag is zero at zero rotor speed)
  - quaternion stays unit length over a long run
  - torque free spin from ω0 = (1, 2, 3) rad/s, not a principal axis, keeps world frame angular momentum R I ω and rotational energy constant. This catches a wrong gyroscopic term and decides RK4 vs Euler
  - mixer signs written down: front left alone faster by F gives τ = (+0.22 F, -0.13 F, +0.016 F)
  - positive pitch (nose down in FLU) at hover thrust accelerates the drone toward +x
  - yawed 90 degrees, a body roll rate turns the drone about world y. Catches q ⊗ ω vs ω ⊗ q
  - motor lag acts on ω: a step up matches 1 - e^(-t/τ) with τ = 0.0125 s at t = 13 ms, and a step down uses τ = 0.025 s
- result: `results/physics.md` with free fall error and the RK4 vs Euler comparison on the spin test

### Phase 2: flight controller (tdd, done 2026-10-15)

Concepts first: PID control, cascaded control loops, saturation and windup.

- `core/controller.{h,cpp}`: rate PI, attitude P on the quaternion error, velocity PI, position P, thrust and tilt limits, an inverse mixer from thrust and torques back to rotor speeds. No D terms in the end, the tests didn't need them. Velocity and acceleration feed forward from the path (added after the route showed 0.65 m of lag on the speed ramps)
- tilt limit 35 degrees, well under acos(1 / 1.61) = 51.6 degrees, the tilt where full throttle only just holds altitude. When the mixer saturates, yaw gives way first (with a 0.016 m moment constant it's the weakest axis), then roll and pitch, and total thrust is kept
- anti windup by capping both integrals. Conditional integration was tried and removed (it made large moves worse, see `results/control.md`)
- tests: mixer round trip and saturation order, hover hold, recovery from 30, 120 and 180 degrees of roll, 1 m steps in x, y, z (up and down) and a 1 rad yaw step (under 10% overshoot, 2% settle under 4 s, 3 s for z), yaw turning the short way through 180 degrees, an x step while facing 90 degrees, no windup after being held, feed forward tracking a 3 m/s² ramp
- `tools/fly_route.cpp`: a reference point moves along a fixed 81.2 m route at 2 and 5 m/s, once with 3 m/s² ramps that stop at each waypoint and once at constant speed with sharp corners. Prints RMS and max distance to the reference and RMS cross track error, plus step responses, recovery from tipped starts and a 10 m step
- result: `results/control.md` with gains, step response numbers, route errors and cost per call

### Phase 3: browser (done 2026-10-15)

Concepts first: fixed timestep and why rendering is decoupled from physics.

- `wasm/bindings.cpp`: one drone, its controller and setpoint. `reset()`, `set_target()` in, `advance(steps)` runs controller plus physics at 1 ms, and the last two states (17 doubles each) are read straight out of wasm memory. Rotor positions come from the C++ parameters so the 3D model can't drift from the physics. Built with the sandbox's Emscripten setup, single thread, 16 MB fixed memory, written into `web/src/wasm/` and committed so a deploy only needs node
- `web/`: Vite + TS + three.js, no React yet (it comes with the panel in phase 6). Z up everywhere. Ground grid, 8 placeholder boxes (phase 4 swaps in the seeded map), the drone with orange front rotors, a ground shadow spot, a target marker
- cameras: a chase camera 3 m behind and 1 m above along the heading with exponential easing, and an orthographic map view with north up and a heading arrow. `c` or the button switches
- loop: fixed 1 ms step, accumulator, 8 ms physics budget per frame, frames capped at 0.1 s, one wasm call per step so waypoints switch on the same step whatever the frame rate (same flight step for step with the same wasm binary), interpolation between the last two states (lerp and slerp)
- demo mission: a 25 m square loop at 8 m, the next waypoint is sent within 1 m, the drone turns to face each leg
- the reviewer found no bugs in the loop, bindings or frame math. Fixed after review: overclaimed CPU cost (now measured the way the page calls it), a wrong comment, determinism wording, the arrival check moved off `Math.hypot`, the first waypoint turning the drone to face south, a tiny negative backlog
- result: `results/browser.md`, 60 fps (the display's rate) and a median 1000 steps per second over 30 s on my Intel Mac, 0.62 to 0.68 µs per step of pure wasm and 0.89 to 0.93 µs the way the page calls it, no console errors on a fresh load

### Phase 4: planning (tdd, done 2026-10-16)

Concepts first: occupancy grids, A*, heuristics, obstacle inflation, path smoothing, replanning.

- `core/map.{h,cpp}`: `city(seed)` puts one building (4 to 8 m on a side, 4 to 18 m tall) on each of 5 × 5 lots of 8 m on a 12 m grid with an 80% chance, home lot empty. Raw `mt19937` draws turned into numbers by hand, so native and wasm builds get the same city. `city_grid()` grows buildings by 0.789 m (0.256 m arm + 0.1 m rotor + 0.433 m half a cell diagonal), blocks every cell whose center is inside, and keeps the ground and ceiling 0.789 m away. 60 × 60 × 20 m at 0.5 m (120 × 120 × 40 cells). `segment_hits()` slab test
- `core/planner.{h,cpp}`: 26 neighbor A* with the straight line heuristic and no corner cutting, `smooth()` line of sight against the grown boxes themselves, `path_blocked()` for replanning
- `core/route.{h,cpp}`: the ramped reference moved out of `fly_route`, plus `follow()` that slides the setpoint along a path, stopping at each waypoint, with steep legs slowed so they never climb or sink faster than 1.5 m/s (3/4 of the controller's cap)
- bindings and page: `new_city()`, `fly_to()` plans from where the drone is and starts following, the page draws the city and the path and loops over five goals. Wasm memory 64 MB
- tests: city determinism and bounds, `mark()` on a hand placed box, grid margins, A* lengths on hand built grids plus a cross check against plain Dijkstra on random grids, walled off goals, smoothing safety on 30 seeded cities, the slab test edge cases, a box dropped on a path, and `follow()` timing, climb limit, heading and repeated points. Mutants caught: corner cutting, overestimating heuristic, equal step costs, no half diagonal, smoothing that ignores buildings, slab edge cases, no climb limit
- what changed from the plan: replanning only exists as pieces (`path_blocked()`, `mark()` and a test). Running it while flying waits for the lidar in phase 5b. Also added the climb limit after the first flights fell up to 7.3 m behind on steep legs
- the reviewer found no safety bugs. Fixed after review: a NaN on repeated path points, the heading resetting to east on a straight up first leg, the page searching again every frame if no goal could be reached, `reset()` leaving the old path running, a wrong reason for the corner cutting rule, replanning described as if it already ran, untested slab edge cases. Left for phase 5b: a blocked start cell makes A* return nothing
- result: `results/planning.md` on my PC. 200 of 200 random trips planned, A* 15.5 ms median natively and 7.6 ms in wasm, smoothed paths 0.5% longer than a straight line at the median, worst tracking error 0.43 m, closest approach 0.68 m from the drone's center (reach 0.356 m), wasm and native paths identical

### Phase 5a: sensors and estimation (tdd, done 2026-10-18)

Concepts first: sensor noise and bias, the Kalman filter, complementary filter, why an accelerometer can't see tilt in flight.

- `core/sensors.{h,cpp}`: IMU (accelerometer and gyro) with white noise, turn on biases and a bias random walk, GPS at 10 Hz, a magnetometer that gives a noisy heading. Bell curve noise drawn by hand from raw `mt19937` draws so native and wasm match. `Noise{}` is a NEO-M8N setup, `rtk()` swaps in a ZED-F9P RTK GPS, `quiet()` turns everything off
- noise levels, all in `results/estimation.md` with their tables: Bosch BMI088 (data sheet BST-BMI088-DS000-19 Rev 1.9) for the IMU, u-blox NEO-M8N (UBX-15031086 R14) and ZED-F9P (UBX-21044850 R05) for GPS, iSentek IST8310 (v1.5) for the magnetometer. Not in the data sheets, and labeled where used: the bias random walk (RotorS IMU defaults) and the NEO-M8N's vertical error (PX4 SITL, twice the horizontal)
- `core/estimator.{h,cpp}`: Kalman filter on position, velocity and accelerometer bias (9 states), with the rotation from the attitude estimate treated as known so it stays linear. Attitude from a PI (Mahony) complementary filter: gyro integration, a weak tilt pull from the raw accelerometer that's only trusted within 3% of 1 g, heading from the magnetometer, and both learn the gyro bias. The gyro is calibrated for 1 s on the pad
- tests: perfect sensors track the truth through a maneuver (2 cm, 5 mrad), the Kalman filter beats raw GPS by more than 3x with real noise, an up and down accel bias is learned, a 1 deg/s yaw gyro bias is learned and yaw doesn't drift, a wrong tilt comes back toward level, even with a big up and down accel bias, and a roll gyro bias is learned. Seven mutants caught
- what changed from the plan: the tilt pull reads the raw accelerometer (otherwise the Kalman filter soaks a tilt error up as fake bias), Q is set above the data sheet to cover model error, and a sideways accel bias turned out impossible to separate from tilt in hover, so only the up and down bias is tested as learned. Flying the planned paths on the NEO-M8N estimate put 51 of 200 flights within the drone's reach of a building, so I use RTK from here on
- result: `results/estimation.md` on my PC. With the NEO-M8N the estimate is 1.08 m RMS off at the median vs 5.15 m for raw GPS. With RTK it's 0.011 m, and all 200 flights are as safe as flying on the truth (closest 0.679 m vs 0.680 m)
- the browser still flies on the true state. The ghost drone and the estimate in the page come with phase 5b

### Phase 5b: lidar and replanning (tdd, done 2026-10-18)

Concepts first: raycasting, turning hits into an occupancy map.

- lidar rays against boxes (slab test) at 10 Hz, hits mark voxels occupied, a blocked path triggers a replan
- tests: a ray hits the nearest face at the right distance, a box dropped on the path is seen and the new path avoids it
- web: estimated drone (ghost) next to the true one, lidar rays, live position error plot (dataviz)
- core done: `core/lidar.{h,cpp}` with the Velodyne VLP-16 pattern (data sheet 63-9229 Rev-K, stepped 2 deg around instead of 0.2), `ray_box()` sharing the slab test, `add_hits()` with one grown box per cell with a hit and known buildings matched within 0.1 m + 0.03 per meter, A* starting from the closest free cell within 2 m when the start is blocked, `tools/lidar_report.cpp`
- result so far: `results/lidar.md`. With a 2 m cube dropped on each of the 200 paths, blind flights all hit it. With the lidar none do, all 200 reach the goal, closest 0.071 m on a 57 degree dive the lidar can't see down (0.112 m before a replan started from the estimate instead of its cell center)
- page: the drone flies on the RTK estimate with the lidar at 10 Hz. The replan rule moved into core as `replan()` (tested, the native tool calls it too). A see-through blue ghost at the estimate, the last turn's hits as dots instead of rays (2,880 lines would hide the scene), a live position error plot over 30 s, and a 2 m cube on the first leg like the native flights, dropped in once the drone is moving so the replan happens where you can see it. No console errors, 1000 steps per second, wasm plans still match native on 200 of 200 queries

### Phase 6: missions and controls

- click or tap to set a goal, drop boxes, wind gust slider, sensor noise toggle, truth vs estimate toggle
- presets: waypoint route, lawnmower search pattern over an area, return to home
- React panel only for controls and readouts (vercel-react-best-practices)

### Phase 7: evaluation and ship

- `tools/batch_eval.cpp`: many seeded maps (the count set by how long it takes), random start and goal, wind on and off. Records success rate, planning time, path length divided by straight line distance, tracking error and estimation error into `results/eval.md` with machine, seeds, map settings, gains, timestep and build flags
- success means within 1 m of the goal, no collision, inside a time limit
- UI pass (impeccable), README with gifs and the numbers table, Netlify deploy, final breakdown update

## How I'll measure things

Every number in the README or the breakdown comes from a file in `results/` that says how it was measured: machine, seed count, map settings, gains, timestep and build flags. If I didn't measure it, it doesn't go in.
