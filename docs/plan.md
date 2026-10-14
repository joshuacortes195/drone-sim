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

### Phase 1: flight physics (tdd)

Concepts to learn first: rigid body dynamics, quaternions, thrust and torque from four motors.

- `core/vecmath.h`: small vec3, quaternion and 3x3 helpers, only what the physics uses
- `core/quad.{h,cpp}`: parameters, state (position, velocity, quaternion, body rates, rotor speeds), motor mixer, derivatives, `step(dt)`
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

### Phase 2: flight controller (tdd)

Concepts first: PID control, cascaded control loops, saturation and windup.

- `core/controller.{h,cpp}`: rate PID, attitude P on the quaternion error, velocity PID, position P, thrust and tilt limits, an inverse mixer from thrust and torques back to rotor speeds
- tilt limit well under acos(1 / 1.61) = 51.6 degrees, the tilt where full throttle only just holds altitude. When the mixer saturates, yaw gives way first (with a 0.016 m moment constant it's the weakest axis), then roll and pitch, and total thrust is kept
- tests: hover hold at a set point, step response in x, y, z and yaw with overshoot, settling time and steady state error inside set limits, no windup after saturating
- `tools/fly_route.cpp`: flies a fixed waypoint route and prints RMS tracking error
- result: `results/control.md` with gains, step response numbers and the route's RMS error

### Phase 3: browser

Concepts first: fixed timestep and why rendering is decoupled from physics.

- Emscripten build and bindings (state out, commands in) using the sandbox's setup
- Vite + TS app, three.js drone, ground and boxes, chase camera and top down view
- fixed physics timestep with an accumulator and per frame budget, render interpolation
- verify with agent-browser: no console errors, fps recorded in `results/browser.md`

### Phase 4: planning (tdd)

Concepts first: occupancy grids, A*, heuristics, obstacle inflation, path smoothing, replanning.

- `core/map.{h,cpp}`: seeded city map with every building below the 20 m ceiling, boxes to voxels. Boxes grow by the rotor arm (0.256 m from center to the farthest rotor) plus the 0.1 m rotor radius plus half a voxel diagonal before voxelizing
- smoothed segments are checked against the grown boxes themselves, not only the voxels
- `core/planner.{h,cpp}`: 26 neighbor A* with a straight line heuristic, line of sight smoothing, replanning when the path hits a newly occupied voxel
- starting size: 60 × 60 × 20 m at 0.5 m voxels (120 × 120 × 40 cells). Adjusted if planning time is too long in the browser
- tests: finds the optimal path on hand built grids, returns no path when the goal is walled off, smoothed path never cuts through a box, the same seed gives the same map
- results: planning time and path length in `results/planning.md`

### Phase 5a: sensors and estimation (tdd)

Concepts first: sensor noise and bias, the Kalman filter, complementary filter, why an accelerometer can't see tilt in flight.

- `core/sensors.{h,cpp}`: IMU (accelerometer and gyro) with white noise and bias random walk, GPS with noise at 10 Hz, a noisy magnetometer for heading. Noise levels taken from real part datasheets, cited in the plan when chosen
- `core/estimator.{h,cpp}`: Kalman filter on position, velocity and accelerometer bias (9 states). The rotation from the attitude estimate is treated as known each step, so it stays a linear (time varying) KF. Attitude from a complementary filter: gyro integration, a low gain tilt correction from the accelerometer and heading from the magnetometer. Without the magnetometer, yaw would drift forever with gyro bias
- the accelerometer measures thrust and rotor drag, not gravity, so in fast flight its x and y readings mostly show rotor drag (Martin and Salaün 2010). The tilt correction is kept weak for that reason
- tests: with zero noise the estimate matches truth in hover, and stays within a set tolerance while maneuvering. The KF position error is lower than raw GPS error. A constant accelerometer bias is learned. Yaw doesn't drift with gyro bias on
- results: estimation RMS error with and without the filter in `results/estimation.md`

### Phase 5b: lidar and replanning (tdd)

Concepts first: raycasting, turning hits into an occupancy map.

- lidar rays against boxes (slab test) at 10 Hz, hits mark voxels occupied, a blocked path triggers a replan
- tests: a ray hits the nearest face at the right distance, a box dropped on the path is seen and the new path avoids it
- web: estimated drone (ghost) next to the true one, lidar rays, live position error plot (dataviz)

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
