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

The leftover time that hasn't been simulated yet goes in an accumulator (I call it the backlog). A 16.7 ms frame runs 16 steps and carries 0.7 ms over to the next frame. Over a second that comes out to 1000 steps, and my readout shows 999 to 1002 per half second window.

The per frame budget is the safety valve. If a frame is slow, the next one owes more steps, which makes it slower too, and the tab locks up. People call that the spiral of death. I stop stepping after 8 ms of physics in one frame and throw away the rest of the backlog, so on a machine that can't keep up the drone flies in slow motion instead of freezing the page. A frame longer than 0.1 s (like switching back to a hidden tab) only counts as 0.1 s, so the drone doesn't jump ahead.

Render interpolation handles the fraction of a step left in the backlog. I keep the last two physics states and draw the drone part way between them: positions blended in a straight line, orientation blended with slerp so it stays a proper rotation. It means the picture is up to one step behind the physics, in exchange for smooth motion. At 1 ms steps the gap is tiny (8 mm at 8 m/s), so here it's more the right habit than a visible fix. It matters a lot more for a 60 Hz or 30 Hz physics step.

In the browser one step (controller plus RK4) costs about 0.67 µs, so a second of flight is under a millisecond of CPU and the page holds the display's 60 fps. The numbers are in `results/browser.md`.

Where it lives: `frame()` and `placeDrone()` in `web/src/main.ts`. The C++ side is `advance()` in `wasm/bindings.cpp`, which keeps the previous state for interpolation. I call it one step at a time and check the mission after each step, so a waypoint switches on the same step no matter how the frames fall. Each call crossing from JavaScript into wasm costs a little, but at 1000 calls a second it doesn't show.

Interview questions:
- Why not just step the physics by the frame time?
- What happens on a machine too slow to keep up, and how do you stop it freezing?
- What does render interpolation cost you, and when does it matter?
- How do you keep a simulation deterministic across different hardware?
