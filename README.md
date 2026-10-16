# drone-sim

An autonomous quadcopter simulator that runs in the browser. The drone has real flight physics, plans its own path through a city of boxes, follows it with a cascaded PID flight controller, sees obstacles with a simulated lidar and works out where it is from noisy sensors. You can set a goal, drop obstacles in its way, add wind and watch it replan.

The physics, control, planning and estimation are written in C++ and tested natively. The same code is compiled to WebAssembly for the browser, where three.js draws the scene.

Status: the flight physics and the flight controller are done and tested. The physics is 6 degrees of freedom with quaternions, the four motor mixer, motor lag and rotor drag, stepped with RK4 at 1 ms. The controller is a cascaded PID (rate, attitude, velocity, position) with an inverse mixer that gives up yaw first when the rotors saturate. It settles 1 m steps in under 1.9 s with under 2% overshoot and follows a ramped 5 m/s route with 0.119 m RMS error. The same C++ runs in the browser as WebAssembly, holding 60 fps on my Intel Mac (i5-8259U) with the physics on a fixed 1 ms step ([results/browser.md](results/browser.md)). Now it also plans: a seeded city of buildings, 26 neighbor A* on a 0.5 m voxel grid with the buildings grown by the drone's size, then line of sight smoothing. Across 200 random trips the smoothed path is 0.5% longer than a straight line at the median, planning takes 7.6 ms in the browser at the median, and when the drone flies each path the worst tracking error is 0.43 m and its center never gets closer than 0.68 m to a building ([results/planning.md](results/planning.md)). In the page it flies a loop of planned paths through the city. Sensors and estimation are next. The full plan is in [docs/plan.md](docs/plan.md), and my notes on the physics and controls are in [docs/robotics-notes.md](docs/robotics-notes.md).

## Build and test

You'll need CMake 3.20+ and a C++17 compiler (I've built it with Apple clang on macOS and MSVC on Windows).

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

To print the physics numbers in [results/physics.md](results/physics.md) (free fall error, RK4 vs Euler drift, cost per step):

```bash
./build/physics_report
```

To print the control numbers in [results/control.md](results/control.md) (step responses, route tracking error, cost per controller call):

```bash
./build/fly_route
```

To print the planning numbers in [results/planning.md](results/planning.md) (planning time, path length, and how each planned path flies):

```bash
./build/plan_report
```

## Run it in the browser

You'll need Node 22+. The compiled wasm is checked in, so this works without Emscripten:

```bash
cd web
npm install
npm run dev
```

The drone flies a loop of planned paths through a seeded city, with the planned path drawn in teal. Press `c` or the button to switch between the chase camera and the map view. Frame rate and physics numbers are in [results/browser.md](results/browser.md).

To rebuild the wasm after changing the C++ (needs [Emscripten](https://emscripten.org)):

```bash
emcmake cmake -S . -B build-wasm
cmake --build build-wasm
```

That writes `web/src/wasm/drone.js` and `drone.wasm`.
