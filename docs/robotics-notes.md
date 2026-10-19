# Robotics notes

My notes on the physics and controls behind the drone sim, written as I build it. Each section says what the idea is in plain words, where it lives in the code and what I'd expect to be asked about it in an interview.

## Coordinate frames

A drone sim has to keep track of two points of view. The world frame is fixed to the ground: x points east, y points north, z points up (people call this ENU). The body frame is glued to the drone: x out the nose, y out the left side, z out the top (FLU). Motor thrust is easy to write in the body frame, since it always points along body z. Gravity and the map are easy to write in the world frame. Most of the math is converting between the two, and the drone's orientation is exactly the rotation that does that conversion.

I picked z up for both frames because it matches the RotorS model I took my parameters from and three.js can be set up with z up too. A lot of aerospace code uses NED (north, east, down) instead, with z pointing down, so thrust is negative there. Mix the two up and you get sign errors that are hard to spot.

Units are SI everywhere: meters, seconds, kilograms, radians. No degrees inside the core.

Where it lives: `docs/plan.md`, Conventions. The code that uses it starts in phase 1.

Interview questions:
- What's the difference between ENU and NED, and why would someone pick one over the other?
- If the drone is pitched forward 10 degrees, which way does the thrust point in the world frame?
- Why keep the physics in SI units and convert only for display?

## Where my airframe numbers come from

I didn't make up the drone. Every physical parameter (mass, inertia, motor constants, rotor positions, motor lag, rotor drag) comes from the Iris model in RotorS, ETH Zurich's open source drone simulator. Iris is a 1.5 kg quad in the same class as an F450. I checked the values against the model file itself, and they're in a table in `docs/plan.md`. There's also a PX4 version of the Iris model with different inertia and motor constants. I used one source only, because mixing two gives a drone that doesn't exist.

Two numbers I can work out from those parameters: it hovers with each rotor at about 660 rad/s, and full throttle gives about 1.6 times its weight in thrust. That's not much headroom, so the controller has to handle saturation.

Interview questions:
- How do you find the hover rotor speed from mass and the thrust constant?
- What does a thrust to weight ratio of 1.6 mean for how hard the drone can maneuver?

## Rigid body dynamics

The drone is one solid object, so two equations move it. Newton's law handles where it goes: add up the forces (four rotor thrusts, gravity, rotor drag), divide by mass, and that's the acceleration of the center of mass. Euler's equation handles how it turns: the torques change the spin rate, but the inertia is different around each axis, so I ω̇ = τ - ω × (I ω). The last term is the part people forget. It's why a spinning object that isn't spinning about one of its main axes wobbles even with zero torque on it. A book flipped in the air tumbles for the same reason.

I write the spin rate ω and the inertia I in the body frame, because the inertia is a fixed number there (it would change every step in the world frame). Position and velocity live in the world frame, so the body thrust gets rotated into the world before I add gravity.

The state is 17 numbers: position (3), velocity (3), orientation quaternion (4), body spin rate (3) and the four rotor speeds (4). The rotor speeds are in the state because the motors can't change speed instantly. Each one chases its command with a first order lag, 12.5 ms speeding up and 25 ms slowing down, which is what the Iris model uses.

To move the state forward one step I use RK4. It samples the derivatives four times across the step and blends them, so it stays accurate when things are spinning fast. The step is a fixed 1 ms.

I measured whether RK4 was worth it. I tumbled the drone from ω = (1, 2, 3) rad/s with the motors off for 60 s. With no torque, the angular momentum seen from the world and the spin energy should never change. RK4 held both to about 1e-12. Semi-implicit Euler drifted 8.8% on the momentum and gained 39% extra energy. It gains instead of losing because the ω update in it is plain explicit Euler (Euler's equation doesn't depend on the orientation), and explicit Euler on a rotation adds a little energy every step. Full numbers are in `results/physics.md`. One RK4 step costs about 0.3 to 0.4 microseconds natively, so speed isn't a reason to drop it.

One simplification: I use the body inertia from the Iris file and leave out the four 5 g rotors. As point masses at the rotor positions they'd add 2.6% to Ixx, 0.8% to Iyy and 1.3% to Izz.

Where it lives: `core/quad.cpp`, `derivative()` and `step()`. The two equations are the `v` and `w` lines in `derivative()`.

Interview questions:
- What is the ω × Iω term, and when is it zero?
- Why write angular velocity and inertia in the body frame?
- Why RK4 instead of plain Euler, and how did you check it mattered?
- With zero torque, what stays constant? (World frame angular momentum and rotational energy. I use that as a test.)

## Quaternions

An orientation could be stored as three angles (roll, pitch, yaw), but Euler angles break at 90 degrees of pitch: two of the axes line up and you lose a direction (gimbal lock). A rotation matrix has no such problem but uses 9 numbers to say 3 things, and they slowly drift apart from being a real rotation. A quaternion uses 4 numbers (w, x, y, z), has no gimbal lock, and the only fix it ever needs is scaling back to length 1.

A rotation by angle a about a unit axis n is q = (cos(a/2), sin(a/2) n). To rotate a body vector into the world I use q v q*. Spinning at body rate ω changes the quaternion at q̇ = ½ q ⊗ (0, ω). The order matters: ω is measured in the body frame, so it goes on the right. Put it on the left and the drone spins about world axes instead, which looks fine while the drone is level and goes wrong as soon as it isn't. I have a test for exactly that.

Where it lives: `core/vecmath.h` (`Quat`, the product, `rotate`), and the `q` line in `derivative()` in `core/quad.cpp`. `step()` renormalizes after every step.

Interview questions:
- What is gimbal lock, and how do quaternions avoid it?
- Why q ⊗ ω and not ω ⊗ q?
- What goes wrong if you never renormalize the quaternion?
- q and -q are the same rotation. Where could that bite you? (In the attitude controller, it can take the long way around. That's phase 2.)

## Thrust and torque from four motors

Each rotor pushes straight up along body z with thrust T = k_f ω², where ω is how fast it spins. Four thrusts add up to the lift. Torque comes from where they push: a rotor at position r makes r × T. With x forward and y left, a rotor on the left side (y > 0) speeding up lifts the left side (positive roll), and a rotor in front (x > 0) speeding up pitches the nose up (negative pitch in my frame).

Yaw is the odd one. A spinning rotor drags on the air, and the motor's push back twists the body the opposite way. That twist is 0.016 m times the thrust. Two rotors spin clockwise and two counterclockwise so the twists cancel at hover. To yaw, speed up one pair and slow the other by the same thrust: lift stays the same and the twists don't cancel. It's the weakest axis, 0.016 m against arms of 0.13 to 0.22 m.

The Iris is not a perfect X. The front arms are 0.22 m out to the side and the back ones 0.20 m. So a pure yaw command also makes a small roll torque, 0.04 times the thrust change. I worked that out on paper and a test checks the exact number.

Two smaller forces only matter when the drone moves through the air. Each rotor gets a drag force against its sideways air speed, -k |ω| v⊥, applied at the rotor, so it also makes a torque (the rotors sit 2.3 cm above the center of mass, so flying forward tips the nose up a little). There's also a tiny rolling moment. Both come straight from the RotorS motor model. Wind will enter through them later, since v⊥ is measured against the air. The sideways speed of each rotor includes the body's own spin (ω × r), so yawing on the spot gets a little drag pushing back, which is how RotorS does it too (it uses each rotor's own velocity).

I matched RotorS term for term, including one quirk: its rolling moment doesn't care which way a rotor spins, so on a quad the four add up instead of cancelling in pairs. Physically, a rotor moving sideways lifts more on the advancing blade, and which side that is depends on spin direction. I kept the RotorS version so every term traces back to one source. At 5 m/s and hover rotor speed it comes to 0.013 N m (the drag test checks that number). RotorS adds the drag and rolling moment as world frame vectors and I add them in the body frame, which comes out the same. The one place I differ: I clamp rotor commands at zero as well as at 838 rad/s, since a real motor on this kind of drone doesn't reverse. RotorS only clamps the top.

The rotor order in the code is the RotorS order: 0 front right (counterclockwise), 1 back left (counterclockwise), 2 front left (clockwise), 3 back right (clockwise).

Where it lives: `wrench()` in `core/quad.cpp` adds up the body force and torque. The rotor table (positions and spin directions) is in `core/quad.h`.

Interview questions:
- Why do quads have two clockwise and two counterclockwise rotors?
- Front left rotor speeds up by F newtons of thrust. What torques does that make? (+0.22 F roll, -0.13 F pitch, +0.016 F yaw on this airframe.)
- Why is yaw authority so weak compared to roll and pitch?
- Thrust goes as ω². What does that mean for a controller that commands thrust?

## PID control

A controller compares where something is with where I want it. The gap is the error, and the controller pushes back against it. The P term pushes in proportion to the error, like a spring. The I term adds the error up over time, so a steady push the P term can't cancel on its own (rotor drag at cruise speed, later wind) gets learned and the error goes to zero. The D term reacts to how fast the error is changing and damps oscillation, like a shock absorber.

My drone ended up with P and I only. Nothing in the tests needed a D term, so I left it out. The motor lag is the slowest thing inside the rate loop (12.5 ms), and with a rate P gain of 20 the loop crosses over around 20 rad/s, which is well below the motors' 80 rad/s. That leaves plenty of phase margin without derivative action.

The I term has a cost I saw in my own numbers. Any error during a transient also gets added up, and once the transient is over that stored integral has to bleed off again. For a PI loop like mine the bleed off has a time constant of kv / ki, which is 2.5 s with my velocity gains. That's why the drone ends a 30 degree recovery test 1.7 cm off at 6 s and then creeps the rest of the way in.

Where it lives: `control()` and `outer_loop()` in `core/controller.cpp`. Gains are in `Gains` in `core/controller.h`.

Interview questions:
- What does each of P, I and D do, and what happens when each one is too high?
- Why doesn't your rate loop have a D term?
- Why does a PI loop sometimes creep slowly toward its target after a disturbance?

## Cascaded control loops

A quad can only push along its own z axis. To move sideways it has to tilt, to tilt it has to spin, and to spin it needs torque. So I stack four controllers, and each one hands the next one its setpoint:

1. position error → velocity setpoint (P)
2. velocity error → acceleration, which turns into a thrust vector, and that gives a tilt and a total thrust (PI)
3. attitude error → body rate setpoint (P)
4. rate error → angular acceleration → torque, through the inertia (PI)

Then the mixer turns total thrust plus torque into four rotor speeds.

Each loop has to be clearly faster than the one outside it, otherwise the outer loop asks for changes before the inner loop can deliver and the whole stack oscillates. My bandwidths step down by roughly 3x each level: rate 20, attitude 7, velocity 2.5, position 1. The rate and attitude loops run every 1 ms physics step. Position and velocity run every 20 steps (50 Hz), which is closer to what a real autopilot does, since GPS updates far slower than the gyro.

I tuned from the inside out. Rate first, then attitude, then velocity and position. Most of the tuning was yaw, which I cover under saturation.

The step from acceleration to attitude is the interesting one. I want the force f = m (a + g). The body z axis has to point along f, so I take the shortest rotation that tips world z onto f and combine it with the heading I want. The total thrust is f projected onto where body z points right now. Then the drone doesn't push hard in the wrong direction while it's still tilting.

That projection gave me my one real control bug. Tipped 90 degrees or more from f, the projection is zero or negative, so the thrust went to zero. With zero thrust the mixer can't make any torque either, so a drone started upside down never turned over and fell 481 m in 10 s. The reviewer found it, and none of my tests started past 30 degrees. Now the thrust never goes below the same 20% floor the outer loop uses, and a test flies it back from 120 and 180 degrees. From upside down it still loses almost 9 m of height, because the floor pushes it down until it's turned over.

One small approximation: I apply the heading before the tilt, so while the drone is tilted its actual heading is a little off from the setpoint (about 1.05 rad for a 1.0 rad setpoint at 35 degrees of tilt on a diagonal). It's exact in hover, so I left it.

Results with these gains: 1 m steps in x, y and z settle into 2% in 1.41 to 1.86 s with 1.0 to 1.6% overshoot. A 1 rad heading change settles in 2.00 s with 2.2% overshoot. The full table is in `results/control.md`.

Where it lives: `outer_loop()` (position, velocity, thrust vector, attitude setpoint) and `control()` (attitude and rate) in `core/controller.cpp`.

Interview questions:
- Why cascade the loops instead of one big controller?
- How far apart should the loop bandwidths be, and what happens if they're too close?
- How do you turn a desired acceleration into a tilt and a thrust?
- Why tune from the inside out?

## Saturation and windup

The rotors have limits: 838 rad/s at the top and zero at the bottom. My airframe only has 1.6 times its weight in thrust, so it hits those limits often. Three things handle that.

First, the outer loop limits what it asks for. The tilt is capped at 35 degrees, which is well under the 51.6 degrees where full throttle only just holds altitude. Total thrust stays between 20% and 90% of full, so there's always some room left to steer. The vertical part of the force gets priority, and the sideways part gets whatever the tilt limit and the leftover thrust allow.

Second, the mixer has a priority order. When the rotors can't do everything, it keeps total thrust first, then as much roll and pitch as fits, then yaw. Yaw goes first because it's the weakest axis (0.016 m of moment against arms of 0.13 to 0.22 m) and losing it for a moment doesn't drop the drone.

Third, anti windup. When the controller asks for more than the drone can give, the integral keeps adding up error it can't fix. When the drone finally gets there, all that stored integral shoves it past the target. I cap both integrals: 2 m/s² for velocity and 1 rad/s² for rate. I picked the rate cap from the physics. Rotor drag at 5 m/s makes about 0.53 rad/s² of pitch, so 1 covers it with room to spare.

I tested windup by holding the drone still for 5 s with the goal 10 m away, then letting go. Its path matched a drone that was never held to under 0.1 mm. With the rate cap at 5 the paths differed by up to 7.6 cm, and with no cap at all they were up to 19.8 m apart.

I also tried conditional integration, which stops the integral from growing while the output is clipped. It made a 10 m move worse (9.54 m after 3 s against 9.90 m without it) because it also stopped the integral from learning the rotor drag push during the sprint. I took it back out.

Yaw taught me the most about saturation. My first yaw gains overshot a 1 rad step by 31%. The yaw torque can only give about 1.5 rad/s² of angular acceleration, so from 1.5 rad/s it needs about 0.75 rad to stop. The attitude loop only starts slowing it 0.5 rad out, so it overshoots no matter what the gains say. Capping yaw rate at 0.8 rad/s fixed it (2.2%).

Where it lives: the limits and caps are in `Gains` in `core/controller.h`. Tilt and thrust limiting are in `outer_loop()`, the integral caps use `clamp3()`, and the priority order is in `mix()`, all in `core/controller.cpp`.

Interview questions:
- What is integrator windup and how do you prevent it?
- When the motors saturate, which axis do you give up first, and why?
- Why limit the tilt angle, and how did you pick 35 degrees?
- Why did your yaw overshoot even with reasonable gains?

## Attitude error from quaternions

The attitude loop needs to know how far the drone still has to turn. With quaternions that's q_err = conj(q) ⊗ q_des, the rotation from where the body is to where it should be, seen from the body. For small angles the x, y, z part of a quaternion is half the angle times the axis, so 2 × (x, y, z) is the rotation still to go, and I multiply that by the attitude gain to get a body rate setpoint.

q and -q are the same orientation, so q_err can come out describing the long way round (270 degrees instead of 90). If q_err.w is negative I flip its sign, and the drone always turns the short way.

Subtracting Euler angles would break at 90 degrees of pitch and jump at ±180 degrees of yaw. The quaternion error has neither problem.

Where it lives: the first lines of `control()` in `core/controller.cpp`.

Interview questions:
- Why use a quaternion error instead of subtracting roll, pitch and yaw?
- What's the double cover problem and how do you handle it?
- Why is twice the vector part a good attitude error?

## The inverse mixer

`wrench()` from phase 1 turns four rotor thrusts into a total thrust and three torques. That's a 4×4 linear map: thrust is the sum, roll is each thrust times its rotor's y, pitch is minus each thrust times its rotor's x, and yaw is minus spin direction × 0.016 × thrust. The controller needs the reverse, so `mix()` solves that same 4×4 system with Gauss Jordan elimination.

I solve the real matrix instead of using the textbook symmetric X formula because my front arms are 0.22 m out and the back ones 0.20 m. The textbook version would add a little roll every time the drone yaws. Solving the real one means a pure yaw request gives pure yaw torque, and the round trip test checks that `wrench()` gives back exactly what `mix()` was asked for.

Saturation works in layers. I solve for the thrust alone, then for the roll and pitch part, and scale that part down until no rotor goes below zero or above full thrust. Then I do the same for yaw with whatever room is left. Thrust is linear in ω², so I mix in thrust and take the square root at the end.

Where it lives: `solve()`, `fit()` and `mix()` in `core/controller.cpp`.

Interview questions:
- How do you go from desired thrust and torques to four motor speeds?
- What changes when the quad isn't perfectly symmetric?
- Why mix in thrust and not in rotor speed?

## Feed forward

Feedback only reacts after an error shows up. Feed forward uses what I already know about the path: if the reference point is moving at 5 m/s and speeding up at 3 m/s², I add that velocity and acceleration straight into the commands, and the feedback loops only fix what's left.

I measured it on my test route (81.2 m with climbs) where the reference speeds up and slows down at 3 m/s² between waypoints. With velocity feed forward only, the RMS error at 5 m/s was 0.652 m. Adding acceleration feed forward brought it to 0.119 m with the same gains. Without it, the velocity loop has to fall 1.2 m/s behind (3 / 2.5) before it asks for the acceleration the ramp needs.

The same route with instant 90 degree corners at constant speed shows a limit. At 5 m/s a corner needs a 7.1 m/s change in velocity, and 35 degrees of tilt gives at most 6.9 m/s², so the drone falls up to 5.5 m behind. That's a reference the drone can't fly. The planner in phase 4 has to smooth corners for that reason.

Where it lives: `Setpoint::vel` and `Setpoint::acc` in `core/controller.h`, used in `outer_loop()`. The route and both reference types are in `tools/fly_route.cpp`, and the numbers are in `results/control.md`.

Interview questions:
- What's the difference between feedback and feed forward, and why use both?
- Why did acceleration feed forward help so much on the ramps?
- What makes a reference trajectory feasible for a quad?

## Fixed timestep and why physics is decoupled from rendering

The browser draws a frame whenever the screen is ready for one. That's about every 16.7 ms on a 60 Hz screen, 7 ms on a 144 Hz one, and whatever a slow laptop manages. If I moved the physics forward by however long the last frame took, the same flight would come out differently on every machine. My controller gains were tuned at exactly 1 ms, and RK4 is only as accurate as its step. So the physics always moves in 1 ms steps, and each frame works out how many of those fit in the real time that has passed.

The leftover time that hasn't been simulated yet goes in an accumulator (I call it the backlog). A 16.7 ms frame runs 16 steps and carries 0.7 ms over to the next frame. Over a second that comes out to 1000 steps, and my steps per second readout stays between 998 and 1002.

The per frame budget is the safety valve. If a frame is slow, the next one owes more steps, which makes it slower too, and the tab locks up. People call that the spiral of death. I stop stepping after 8 ms of physics in one frame and throw away the rest of the backlog, so on a machine that can't keep up the drone flies in slow motion instead of freezing the page. A frame longer than 0.1 s (like switching back to a hidden tab) only counts as 0.1 s, so the drone doesn't jump ahead.

Render interpolation handles the fraction of a step left in the backlog. I keep the last two physics states and draw the drone part way between them: positions blended in a straight line, orientation blended with slerp so it stays a proper rotation. It means the picture is up to one step behind the physics, in exchange for smooth motion. At 1 ms steps the gap is tiny (8 mm at 8 m/s) and you can't see the difference. It matters a lot more for a 60 Hz or 30 Hz physics step.

In the browser one step (controller plus RK4) costs about 0.65 µs of pure wasm, and about 0.9 µs the way the page calls it, one step at a time with the mission and budget checks. So a second of flight is under a millisecond of CPU, and on my Intel Mac (i5-8259U) the page holds the display's 60 fps. The numbers are in `results/browser.md`.

Where it lives: `frame()` and `placeDrone()` in `web/src/main.ts`. The C++ side is `advance()` in `wasm/bindings.cpp`, which keeps the previous state for interpolation. I call it one step at a time and check the mission after each step, so a waypoint switches on the same step no matter how the frames fall. With the same wasm binary the flight comes out the same step for step at any frame rate. I haven't checked it against the native build, and it may not match to the last bit since the browser build uses a different math library. When the budget drops steps the sim falls behind the wall clock, but the steps it does run are the same. Calling in once per step costs about 40% more than one big batch.

Interview questions:
- Why not just step the physics by the frame time?
- What happens on a machine too slow to keep up, and how do you stop it freezing?
- What does render interpolation cost you, and when does it matter?
- How do you keep a simulation deterministic across different frame rates, and where does that guarantee stop?

## Occupancy grids and growing the obstacles

My planner doesn't look at buildings directly. It cuts the 60 x 60 x 20 m flying area into 0.5 m cubes, 120 x 120 x 40 = 576,000 cells, and marks each one free or blocked. That turns "fly through a city" into "find a way through a 3D maze", which is a problem search algorithms already solve well. A finer grid fits tighter gaps but costs memory and search time: 0.1 m cells would be 125 times as many.

The drone isn't a point, so before marking cells I grow every building by how far the drone reaches plus a bit. The reach is 0.256 m from the center to the farthest rotor plus the 0.1 m rotor radius. On top of that goes half a cell diagonal (0.433 m), because a cell only counts as free by its center, and the drone can be anywhere in that cell. That's 0.789 m in total. After that I can plan for a point: any point inside a free cell is at least 0.789 - 0.433 = 0.356 m from a real wall, which is exactly the drone's reach. The ground and the 20 m ceiling are kept the same distance away, so the lowest the drone plans is 1.25 m up.

Where it lives: `city()`, `mark()` and `city_grid()` in `core/map.cpp`, the margins at the top of `core/map.h`.

Interview questions:
- Why plan on a grid instead of with the boxes themselves?
- Why grow the obstacles, and by how much?
- What does the cell size trade off?

## A* and heuristics

A* explores outward from the start, but it always expands the open cell with the lowest g + h next. g is the distance flown to reach that cell, and h is a guess at the distance left. Without h it's Dijkstra's algorithm, which spreads out evenly in every direction. With a good h it leans toward the goal and opens far fewer cells. A priority queue keeps the cheapest open cell on top.

The guess has rules. Admissible means h never overestimates the distance left, and that's what guarantees the path A* returns is the shortest one. Consistent means h never drops by more than the cost of the step you just took, so a cell is never opened twice. The straight line distance to the goal is both, since nothing is shorter than a straight line. If h overestimates, A* gets faster but greedy, and it can miss the shortest path. I proved this one to myself: in my tests, multiplying h by 1.5 makes A* disagree with a plain Dijkstra search on random cluttered grids.

Each cell touches 26 others: 6 through a face (0.5 m away), 12 through an edge (0.71 m) and 8 through a corner (0.87 m). Using the real length of each step keeps diagonal moves honest. With only 6 neighbors, paths turn into staircases up to 1.73 times too long. I also don't let a diagonal step squeeze past a blocked cell. Strictly, the margin already covers that: any point on a step between two touching cells is within half a cell diagonal of a free cell center, so the drone still stays its full reach away. The reviewer checked it with the rule removed and the closest any raw path came was 0.783 m. I kept the rule anyway since it costs nothing, and my hand worked wall test depends on it.

Where it lives: `astar()` in `core/planner.cpp`.

Interview questions:
- What's the difference between A* and Dijkstra?
- What makes a heuristic admissible, what makes it consistent, and what happens if it isn't?
- Why 26 neighbors, and why ban cutting corners?

## Smoothing the path

A grid path is jagged. Across 200 seeded cities, the raw A* path was 10% longer than a straight line at the median and had 68 waypoints. Line of sight smoothing walks the path and keeps only the points it has to: from each kept point it jumps to the farthest point further down the path that it can reach in a straight line without touching a grown building. That brought the median down to 3 waypoints and 0.5% longer than a straight line.

The check runs against the grown boxes themselves with a segment vs box slab test, not against the cells, so a shortcut can't slip between two blocked cells. The slab test treats the segment as a + t (b - a) for t from 0 to 1. For each axis it works out the range of t where the segment is between the box's two faces on that axis. If the three ranges overlap, the segment is inside the box for that stretch. The smoothed path isn't the true shortest path, but it's close. Any angle planners like Theta* get closer still.

Where it lives: `smooth()` in `core/planner.cpp`, `segment_hits()` in `core/map.cpp`. The numbers are in `results/planning.md`.

Interview questions:
- Is the smoothed path optimal?
- Why check shortcuts against the boxes and not the grid?
- How does the slab test work?

## Replanning and flying the path

The plan for phase 5b: when the lidar sees a new obstacle, check whether any segment of the path still ahead now touches it (grown by the drone's reach). If one does, mark the new box in the grid and run A* again from where the drone is. Right now only the pieces exist: `path_blocked()` and `mark()`, and a test that drops a box on a path, sees it, and plans around it. Nothing calls them while flying yet. One gap to close first: if the new box is close enough to block the drone's own cell, A* has no free start and returns nothing, so the drone would just hold. At this map size a full A* took 15 ms at the median in the native build on my PC, so plain replanning is enough. Incremental planners like D* Lite exist for when replanning from scratch is too slow, and I'd measure before reaching for one.

Flying the path uses the ramped reference from phase 2. A setpoint slides along each leg, speeding up and slowing down at 3 m/s² with a 5 m/s cruise, and stops on every waypoint, with its velocity and acceleration fed forward. Stopping at each corner means the drone never has to make the instant 90 degree turn that cost about 5 m of error at 5 m/s. Smoothing is what makes that affordable: 3 stops instead of 68. The nose points along the current leg, and a straight up or down leg keeps the heading it had.

Steep legs need one more rule. My controller caps vertical speed at 2 m/s, but a leg that drops 17 m over 20 m at 5 m/s asks for 4.2 m/s down. The drone couldn't keep up and fell 7.3 m behind the setpoint. Now a leg slows down whenever its up or down part would pass 1.5 m/s, which is 3/4 of the cap so the feedback still has room to catch up. Across 200 planned flights, the worst distance to the setpoint went from 7.34 m to 0.43 m. So the path follower has to know the controller's limits, or it hands the drone a reference it can't fly.

Where it lives: `path_blocked()` in `core/planner.cpp`, `follow()` and the climb limit in `core/route.cpp`, and `fly_to()` in `wasm/bindings.cpp`. Flight numbers are in `results/planning.md`.

Interview questions:
- When do you replan, and why not use D* Lite?
- How do you turn a list of waypoints into something a flight controller can track?
- Why did steep legs fall behind, and how did you fix it?

## Sensor noise and bias

Every reading is the truth plus a bias plus noise. Noise is new on every sample and averages away. Bias is an offset that stays put, so averaging can't touch it, and it slowly wanders as a random walk: every step adds a tiny random nudge, so after time t it has moved about walk × √t. A gyro that's 1 degree per second off, which is the BMI088's datasheet turn on offset, would turn the heading 60 degrees in a minute if nothing corrected it.

Datasheets give white noise as a density, like 0.014 deg/s/√Hz for the BMI088 gyro. The spread of one sample is that density times the square root of the sample rate, so at 1000 samples a second the gyro noise per sample is 0.014 × √1000 = 0.44 deg/s. Sampling faster makes each sample noisier but doesn't change how much noise piles up once you integrate.

My noise numbers come from three real parts: a Bosch BMI088 IMU, a u-blox NEO-M8N GPS and an iSentek IST8310 magnetometer. The GPS datasheet gives 2.5 m CEP50, the radius that holds half the fixes. For a round bell curve that's 1.1774 σ, so each axis gets σ = 2.12 m. The bias random walk isn't in the BMI088 datasheet, so I used the RotorS simulator's IMU defaults and wrote that down.

I draw the bell curve numbers myself from raw mt19937 draws (Box-Muller), because the standard library's normal distribution gives different numbers on MSVC and in the browser build.

Where it lives: `core/sensors.{h,cpp}`. Sources are in `docs/plan.md`.

Interview questions:
- What's the difference between noise and bias, and which one hurts more?
- How do you turn a noise density into the noise on one sample?
- What does CEP mean, and how do you get a standard deviation from it?

## What an accelerometer actually measures

An accelerometer measures specific force: every push except gravity, divided by mass. Sitting on a table it reads 9.81 straight up. In free fall it reads zero. On my drone that's thrust plus rotor drag over the mass, which is exactly `wrench(P, s).force / P.mass` in the body frame.

In a steady hover that reading points straight up, so it tells me which way is down and I can correct tilt with it. But when the drone tilts to speed up, the thrust tilts with it and the accelerometer still reads roughly "straight up the body". It can't see the tilt. In fast flight its sideways part is mostly rotor drag, not tilt (Martin and Salaün, 2010).

In a hover, a sideways accelerometer bias also looks exactly like a tilt. A 0.2 m/s² bias reads the same as being tilted 0.2 / 9.81 = 20 mrad. Nothing in hover can tell those two apart, so my attitude estimate is about that far off, 9 to 30 mrad at worst over four seeded flights. The up and down bias is different: GPS height catches it, and my filter learns it.

Interview questions:
- What does an accelerometer read in free fall?
- Can you get attitude from an accelerometer on a multirotor?
- Why can't you tell a sideways accel bias from a tilt?

## The Kalman filter

My Kalman filter tracks 9 numbers: position, velocity and the accelerometer bias, each in 3D. Along with them it keeps a 9 by 9 matrix P that says how unsure it is about each number and how those doubts are linked.

Predict runs every 1 ms with the IMU. It takes the accelerometer reading minus the bias, turns it into the world frame with the attitude estimate, adds gravity and integrates velocity and position. P grows, because the IMU is noisy. Q, the process noise, says by how much.

Correct runs at 10 Hz with GPS. It compares the fix with the predicted position. The Kalman gain K decides how far to move toward the fix: far when I'm unsure and GPS is good, a little when it's the other way around. R, the measurement noise, is how much I distrust GPS. Then P shrinks. The bias gets learned for free: if the IMU keeps predicting positions that GPS keeps pulling the same way, P has linked that error to the bias and K corrects it.

It stays a linear Kalman filter because I treat the rotation from the attitude filter as known at each step. A full error state EKF would put attitude in the filter too. I left that out of scope.

I set Q's accelerometer noise to 0.01 m/s²/√Hz, well above the BMI088's 0.0016. Q has to cover what my model gets wrong as well as sensor noise, like reading the IMU once per step and the tilt pull below. With perfect sensors that change took the worst position error through a maneuver from 11 cm to 1 cm (measured with an earlier tilt setting), and with real noise it changed almost nothing.

With real noise, over 200 planned flights, the estimate was 1.08 m RMS off at the median while raw GPS was 5.15 m off, so about 5 times better.

Where it lives: `predict()` and `gps()` in `core/estimator.cpp`.

Interview questions:
- What are Q and R, and what happens if you set them wrong?
- What does the Kalman gain do?
- Why is it linear if the drone rotates?
- How does the filter learn a bias it never measures directly?

## The complementary filter

Attitude comes from a complementary filter. The gyro gets integrated every step, which is fast and smooth but drifts. Two slow references pull it back: a weak nudge toward the tilt the accelerometer suggests, and a nudge toward the heading the magnetometer reads. The fast part comes from the gyro and the slow part from the references, which is why it's called complementary. Without the magnetometer nothing would ever fix yaw, because gravity says nothing about which way north is.

Mine is the PI version (Mahony). Part of each correction also goes into a learned gyro bias. With a 1 deg/s yaw gyro bias the gyro alone would be 1.6 rad off after 90 s. Mine ended 0.0015 rad off and learned the bias to within 0.0003 rad/s.

Three things I learned tuning it:
- The tilt pull only trusts the accelerometer when it reads within 3% of 1 g, after taking off the up and down bias the Kalman filter learned (checking the raw reading kept the pull switched off for a whole flight when the z bias was big). Even then the pull is weak (0.03 rad/s per rad). Stronger made the estimate lean toward level during hard maneuvers.
- The integral gain has to match the pull for good damping, ki = (kp / 1.4)² for 0.7. My first pick was underdamped, and a wrong tilt took ages to come back.
- The tilt pull uses the raw accelerometer, not the Kalman filter's bias corrected one. Otherwise, with a wrong tilt, the Kalman filter learns the fake sideways push as accelerometer bias within seconds, the bias corrected reading then agrees with the wrong tilt, and nothing fixes it. A 0.05 rad error only came down to 0.038 in 100 s that way. With the raw reading it came down to 0.010.

Real flight controllers also calibrate the gyro on the pad: they average a second of readings while the drone sits still. I do the same. A weak tilt pull can't keep up with a 1 deg/s roll bias from a cold start.

Where it lives: `predict()` and `mag()` in `core/estimator.cpp`. Tests are in `tests/test_estimator.cpp`.

Interview questions:
- Why is it called complementary?
- Why do you need a magnetometer?
- How did you pick the gains?
- Why does the tilt correction use the raw accelerometer?

## Flying on the estimate

Once the controller flies on the estimate instead of the truth, everything downstream inherits the estimate's error. I flew the same 200 planned paths three ways. On the truth, no flight came within the drone's 0.356 m reach of a building. On the estimate with a regular NEO-M8N GPS, 51 of 200 did, and one went inside a building. The filter itself was doing fine: 1.08 m RMS error vs 5.15 m for raw GPS. But the planner only keeps 0.789 m from buildings in 4 m wide streets, and that margin assumes the drone knows where it is.

Growing the margin by 2 to 3 m would close most of the streets, so I fixed it with better position data instead. An RTK GPS (u-blox ZED-F9P) corrects its fixes with a base station nearby and gets about a centimeter. With it the estimate was 1.1 cm RMS off, and the flights were as safe as on the truth: the closest any of them got was 0.679 m vs 0.680 m. So I use RTK from here on, and the regular GPS stays as the comparison. Real drones that fly close to things use RTK, lidar or cameras for the same reason.

Where it lives: `rtk()` in `core/sensors.cpp`, the flights in `tools/plan_report.cpp`, the numbers in `results/estimation.md`.

Interview questions:
- Your filter cut the GPS error 5x. Why did the drone still hit buildings?
- How would you size an obstacle margin from estimation error?
- What does RTK do differently from regular GPS?

## Lidar: raycasting against boxes

A lidar spins a laser and times each pulse's bounce, which gives the distance to the first thing in that direction. To fake one I shoot a ray from the drone and find the nearest box face it goes into. I already had the slab test from phase 4: a box is where three pairs of parallel walls overlap, so I work out when the ray is between each pair and keep the part where all three overlap. The start of that overlap is the distance. I model a Velodyne VLP-16: 16 beams from 15 degrees down to 15 degrees up, all the way around, 10 times a second, with 3 cm of range noise.

One detail I like: the range comes from where the drone really is, because that's physics, but the drone places the hit using where it thinks it is. With RTK that's about a centimeter off in position, but the attitude estimate is off by up to 0.027 rad, and that moves a hit 30 m away by 0.8 m. So far hits are smeared, and the code has to expect it.

Where it lives: `ray_box()` and the shared `slab()` in `core/map.cpp`, `scan()` in `core/lidar.cpp`, the data sheet values in `results/lidar.md`.

Interview questions:
- Why does the slab test work, and what happens with a ray parallel to a face?
- How would you make this fast with thousands of objects?
- Why use the true pose for the range and the estimate for placing the hit?

## Turning hits into a map

The planner thinks in cells, with every obstacle grown by the drone's margin so the drone can be a dot. So each hit has to become blocked cells, and also a box, because smoothing and the replan check work on boxes. My first version skipped any hit whose cell was already blocked. That sounded smart but it was wrong: a cell can be blocked just because it's inside the margin of a neighboring hit, and then the side of a box seen after its front never turned into boxes. The drone flew within 1 cm of a box without noticing. Now there's one box per cell that has a hit, that cell grown by the full margin, which covers every hit inside it.

Hits on buildings the drone already knows have to be thrown out, or they'd pile up as fake obstacles. A hit counts as a known building if it's within 0.1 m plus 0.03 m per meter of range, since attitude error grows with distance. Without the range part, fake obstacles blocked the goal on 20 of 200 flights.

Where it lives: `add_hits()` in `core/lidar.cpp`, the hit flag in `Grid` in `core/map.h`.

Interview questions:
- Occupancy grid or a list of obstacles, and why do you keep both?
- How do you tell a new obstacle from a known one with a noisy pose?
- What breaks if you inflate a hit point instead of its cell?

## Replanning when the lidar sees something

Replanning every scan would waste time, so the drone only replans when the boxes from this scan cut the path it's flying. Then it plans from the estimate to the same goal. If it's already inside the margin of something it just saw, its own cell is blocked, so A* now starts from the closest free cell within 2 m. Across 200 trips with an unknown cube dropped on the path, the blind drone hit the cube every time. With the lidar none of them did, all 200 reached the goal, and the worst one came 0.112 m from the cube. That one was diving at 57 degrees, steeper than the lidar can look down.

Where it lives: `tools/lidar_report.cpp`, the blocked start in `astar()` in `core/planner.cpp`, the numbers in `results/lidar.md`.

Interview questions:
- When do you replan, and how do you avoid replanning all the time?
- What happens when the new obstacle closes off every way through?
- What are the blind spots of a 16 beam lidar on a drone, and how would you cover them?
