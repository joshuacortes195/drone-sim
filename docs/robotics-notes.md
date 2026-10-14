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
