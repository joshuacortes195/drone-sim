# drone-sim

An autonomous quadcopter simulator that runs in the browser. The drone has real flight physics, plans its own path through a city of boxes, follows it with a cascaded PID flight controller, sees obstacles with a simulated lidar and works out where it is from noisy sensors. You can set a goal, drop obstacles in its way, add wind and watch it replan.

The physics, control, planning and estimation are written in C++ and tested natively. The same code is compiled to WebAssembly for the browser, where three.js draws the scene.

Status: the flight physics is done and tested (6 degrees of freedom, quaternions, the four motor mixer with motor lag and rotor drag, RK4 at 1 ms). The flight controller is next. The full plan is in [docs/plan.md](docs/plan.md), and my notes on the physics and controls are in [docs/robotics-notes.md](docs/robotics-notes.md).

## Build and test

You'll need CMake 3.20+ and a C++17 compiler.

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

To print the physics numbers in [results/physics.md](results/physics.md) (free fall error, RK4 vs Euler drift, cost per step):

```bash
./build/physics_report
```
