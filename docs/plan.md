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
- IMU (accelerometer and gyro with noise and bias), GPS with noise, lidar raycasts
- linear Kalman filter on position and velocity, complementary filter for attitude. The controller flies on the estimate. A truth mode flies on the real state for comparison
- wind as a steady breeze plus gusts
- native batch evaluator over many seeded maps
- desktop first web app. On a phone it loads, flies the default mission and takes a tapped goal

Out, unless I decide otherwise later:

- RRT star (only if I ask for it)
- full error state EKF with attitude and bias states
- other obstacle shapes, moving obstacles, multiple drones
- battery model, ground effect, blade flapping, aerodynamics past rotor drag
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
| rotor drag coefficient | 8.06428e-05 | per rotor force = -k ω v, v = air relative velocity in the rotor plane |
| rolling moment coefficient | 1e-06 | |

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

- physics: fixed 1 ms step. RK4 on the rigid body plus motor lag, quaternion renormalized after each step. The phase 1 free fall and torque free spin tests decide whether RK4 is needed or semi implicit Euler is enough
- controller: rate loop every physics step, outer loops slower (start at 50 Hz for position and velocity, tuned in phase 2)
- sensors: IMU every physics step, GPS at 10 Hz, lidar at 10 Hz (settled in phase 5)
- browser: physics steps on a fixed clock with an accumulator and a per frame budget, rendering interpolates between states. Same pattern as the gravity sandbox

## Milestones

Each phase is one session, ends with tests passing, a commit, a reviewer pass, a robotics notes update and a breakdown update.

### Phase 0: plan (done 2026-10-14)

Scope questions, this plan, the rules file outside the repo, a CMake + doctest skeleton.

### Phase 1: flight physics (tdd)

Concepts to learn first: rigid body dynamics, quaternions, thrust and torque from four motors.

- `core/math.h`: small vec3, quaternion and 3x3 helpers, only what the physics uses
- `core/quad.{h,cpp}`: parameters, state (position, velocity, quaternion, body rates, rotor speeds), motor mixer, derivatives, `step(dt)`
- tests:
  - equal rotor speeds whose thrust matches the weight hold a hover (no drift, no rotation over 10 s)
  - spinning the clockwise pair faster yaws it the expected way and doesn't roll, pitch or climb off
  - motors off, it falls as z0 - g t² / 2 (rotor drag is zero at zero speed, so this is exact)
  - quaternion stays unit length over a long run
  - torque free spin keeps angular momentum (catches a wrong gyroscopic term)
  - a motor step reaches 63% of the target after one time constant
  - mixer: one rotor faster gives roll and pitch torques with the right signs
- result: `results/physics.md` with free fall error and the RK4 vs Euler comparison

### Phase 2: flight controller (tdd)

Concepts first: PID control, cascaded control loops, saturation and windup.

- `core/controller.{h,cpp}`: rate PID, attitude P on the quaternion error, velocity PID, position P, thrust and tilt limits, an inverse mixer from thrust and torques back to rotor speeds
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

- `core/map.{h,cpp}`: seeded city map, boxes to voxels, boxes inflated by the drone radius plus a margin
- `core/planner.{h,cpp}`: 26 neighbor A* with a straight line heuristic, line of sight smoothing, replanning when the path hits a newly occupied voxel
- starting size: 60 × 60 × 20 m at 0.5 m voxels (120 × 120 × 40 cells). Adjusted if planning time is too long in the browser
- tests: finds the optimal path on hand built grids, returns no path when the goal is walled off, smoothed path never cuts through a box, the same seed gives the same map
- results: planning time and path length in `results/planning.md`

### Phase 5: sensing and estimation (tdd)

Concepts first: sensor noise and bias, raycasting, the Kalman filter, complementary filter.

- `core/sensors.{h,cpp}`: IMU with white noise and bias random walk, GPS with noise at 10 Hz, lidar rays against boxes (slab test). Noise levels taken from real part datasheets, cited in the plan when chosen
- `core/estimator.{h,cpp}`: KF on position and velocity, attitude complementary filter
- lidar hits mark voxels occupied, a blocked path triggers a replan
- tests: zero noise estimate matches truth, the KF error shrinks below GPS noise, a box dropped on the path is seen and the new path avoids it
- web: estimated drone (ghost) next to the true one, live position error plot (dataviz)
- results: estimation RMS error with and without the filter in `results/estimation.md`

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
