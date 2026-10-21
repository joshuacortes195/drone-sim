# drone-sim

An autonomous quadcopter simulator that runs in the browser. The drone has real flight physics, plans its own path through a city of boxes, follows it with a cascaded PID flight controller, sees obstacles with a simulated lidar and works out where it is from noisy sensors. You can tap to give it a goal, drop boxes in its way and turn on wind. Mission presets and a graphics pass come next (phase 6b in the plan).

The physics, control, planning and estimation are written in C++ and tested natively. The same code is compiled to WebAssembly for the browser, where three.js draws the scene.

Status: phases 0 to 5 of 7 are done.

The physics is 6 degrees of freedom with quaternions, a four motor mixer, motor lag and rotor drag, stepped with RK4 at 1 ms. The controller is a cascaded PID (rate, attitude, velocity, position) with an inverse mixer that gives up yaw first when the rotors saturate. It settles 1 m steps in under 1.9 s with under 2% overshoot and follows a ramped 5 m/s route with 0.119 m RMS error ([results/control.md](results/control.md)). In the browser it held 60 fps on my Intel Mac (i5-8259U) with the physics on a fixed 1 ms step ([results/browser.md](results/browser.md)).

For planning, a seed builds a city of buildings, and 26 neighbor A* searches a 0.5 m voxel grid with the buildings grown by the drone's size, then line of sight smoothing cuts the path down. Across 200 random trips the smoothed path is 0.5% longer than a straight line at the median and takes 7.6 ms to plan in the browser. Flying each one, the worst tracking error is 0.43 m and the drone's center never gets closer than 0.68 m to a building ([results/planning.md](results/planning.md)).

The drone works out where it is from simulated sensors with noise from real data sheets, using a Kalman filter for position and a complementary filter for attitude. With an RTK GPS the estimate is 1.1 cm RMS off and the planned flights are as safe as flying on the truth. A regular GPS puts 51 of 200 flights within reach of a building ([results/estimation.md](results/estimation.md)).

It also has a simulated lidar (the Velodyne VLP-16 beam pattern) and replans when it sees something new. I dropped an unknown cube on each of 200 planned paths: flying blind hit it every time, and with the lidar every flight got around it and reached the goal ([results/lidar.md](results/lidar.md)).

Wind is a steady breeze plus gusts, and it pushes on the drone through rotor drag. In an 8 m/s breeze it leans 6.5 degrees into the wind and holds its spot within 0.3 mm. In 3 m/s gusts on RTK it wanders 0.15 m RMS and gets pushed at most 0.42 m. With a regular GPS it wanders about 1.25 m whatever the wind, so the GPS matters more than the wind ([results/wind.md](results/wind.md)).

In the page the drone flies on its own estimate, with a see-through blue ghost where it thinks it is, a live plot of how far off that is, and the lidar hits as dots. The full plan is in [docs/plan.md](docs/plan.md), and my notes on the physics and controls are in [docs/robotics-notes.md](docs/robotics-notes.md).

## Build and test

You'll need CMake 3.20+ and a C++17 compiler (I've built it with Apple clang on macOS and MSVC on Windows).

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

On Windows I build with the Visual Studio 2022 Build Tools. From an "x64 Native Tools Command Prompt" (or after running `vcvars64.bat`), the same commands work with Ninja, and the programs end in `.exe`:

```bat
cmake -S . -B build -G Ninja
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

To print the lidar numbers in [results/lidar.md](results/lidar.md) (200 flights past an unknown cube, blind and with the lidar, about 30 s):

```bash
./build/lidar_report
```

To print the wind numbers in [results/wind.md](results/wind.md) (540 hovers in breezes and gusts on the truth, RTK and a regular GPS, about 90 s):

```bash
./build/wind_report
```

## Run it in the browser

You'll need Node 22+. The compiled wasm is checked in, so this works without Emscripten:

```bash
cd web
npm install
npm run dev
```

The drone flies a loop of planned paths through a seeded city, with the planned path drawn in teal. The blue ghost is where the drone thinks it is, the magenta dots are the last lidar turn, and the outlined cube is one it wasn't told about. It drops onto the first leg once the drone is moving, so you can watch the lidar find it and the path get redrawn, with the old path faded for 2 s. The plot in the corner is how far the estimate is from the real drone, averaged over each 0.1 s. Press `c` or the button to switch between the chase camera and the map view.

The panel on the side lets you tap the city to set a goal (it stops the tour until you reset) or drop a 2 m box the drone doesn't know about, set a breeze from the west and gusts, switch to a regular GPS and fly on the real position instead of the estimate. If the drone hits something it stops and a banner offers a reset. Frame rate and physics numbers are in [results/browser.md](results/browser.md).

To rebuild the wasm after changing the C++ (needs [Emscripten](https://emscripten.org)):

```bash
emcmake cmake -S . -B build-wasm
cmake --build build-wasm
```

On Windows add `-G Ninja` to the first line.

That writes `web/src/wasm/drone.js` and `drone.wasm`.
