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
