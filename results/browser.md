# Browser results (phase 3)

Measured 2026-10-15 on the Vite dev server with agent-browser driving a headed Chrome.

## How

- machine: Intel Core i5-8259U @ 2.30GHz, 8 GB RAM, Intel Iris Plus Graphics 655 (WebGL through ANGLE Metal), macOS 15.7.7, 60 Hz display
- browser: Chrome 155 (agent-browser 0.39.1, `--headed`), viewport 1200 × 610 at device pixel ratio 2
- wasm: emcc 6.0.12, CMake Release (`-O3 -DNDEBUG`), single thread, no SIMD, double precision, 16 MB fixed memory
- page: three.js with antialiasing, 8 placeholder boxes, the drone flying the 25 m square demo loop at 8 m altitude on the chase camera
- loop: fixed 1 ms physics step called one step at a time from JavaScript, accumulator, 8 ms physics budget per frame, frames over 0.1 s counted as 0.1 s, render interpolation between the last two states
- the page isn't cross origin isolated (no threads needed), so Chrome rounds `performance.now()` coarsely. That's why the per step cost below comes from one big batch instead of the per frame readout

## Frame rate and physics rate

The readout in the corner averages over half a second. I sampled it every 500 ms for 30 s on the chase camera, twice. Run A was the commit before the review fixes (same loop, arrival check with `Math.hypot`), with only this page open. Run B was the final code, while another project's GPU heavy page was open in a second browser on the same machine. The map view wasn't measured.

| | run A (58 windows) min / median / max | run B (59 windows) min / median / max |
|---|---|---|
| frames per second | 60 / 60 / 60 | 52 / 60 / 60 |
| physics steps per second | 999 / 1000 / 1002 | 998 / 1000 / 1002 |
| physics time per frame (ms, coarse timer) | 0.02 / 0.04 / 0.09 | 0.01 / 0.03 / 0.05 |

60 fps is the display's refresh rate (the readout rounds to whole frames). In run A every window read 60. In run B one window dropped to 52, with the other page sharing the GPU. Either way the physics kept real time: a median of 1000 steps of 1 ms each per second. No console errors or page errors on a fresh load. (Rebuilding the wasm while the dev server is open does throw a few errors: Vite reloads the page while Emscripten is part way through rewriting `drone.js` and `drone.wasm`, so for a moment they don't match. A reload after the build fixes it.)

## Cost of one step in the browser

Two ways, each timed over 100,000 steps of 1 ms (controller plus RK4 physics) from hovering at 10 m with a target 22.4 m away, in the page on the final code:

- batch: one `advance(100000)` call, the pure wasm cost
- loop: the way the page actually runs it, one `advance(1)` call per step plus the arrival check and the `performance.now()` budget check

| run | batch µs per step | loop µs per step |
|---|---|---|
| 1 | 0.646 | 0.929 |
| 2 | 0.642 | 0.905 |
| 3 | 0.630 | 0.903 |
| 4 | 0.624 | 0.885 |
| 5 | 0.661 | 0.917 |

An earlier batch run on the same wasm gave 0.661 to 0.680 µs. Calling in one step at a time costs about 40% more than the batch, so the page spends a bit under 1 ms of CPU per real second on physics, well under the 8 ms per frame budget. The per frame readout (median 0.03 to 0.04 ms, so 1.8 to 2.4 ms per second) reads higher than that, but it comes from the coarse timer and I don't trust it at this size. Natively the two parts were measured separately: 308 to 394 ns per RK4 step (`results/physics.md`) and 268 to 300 ns per `control()` call (`results/control.md`).

## Download size

From `npm run build`:

| file | size | gzip |
|---|---|---|
| `drone.wasm` | 19.89 kB | 10.52 kB |
| page js (three.js plus my code) | 564.21 kB | 142.66 kB |
| `index.html` | 0.87 kB | 0.53 kB |

## Phase 5b page: flying on the estimate with the lidar (my PC)

Checked 2026-10-18 on my PC (AMD Ryzen 7 3700X, 16 GB RAM, Windows 10 Home 19045), headless Chrome 155 driven by Playwright, the Vite dev server, wasm from emcc 6.0.12. Not comparable with the Mac numbers above.

Each physics step now also runs the IMU and the estimator, with GPS and magnetometer every 100 steps and a full lidar turn (2,880 rays) plus the replan check every 100 steps.

| | value |
|---|---|
| console errors on a fresh load and 62 s of flying | none |
| frame rate (readout every 2.5 s, 24 readings) | 180 fps every reading, probably the display's refresh rate, not checked |
| physics steps per second | 999 to 1001 every reading |
| physics per frame (performance.now, coarse at this size) | 0.08 to 0.18 ms |
| replans in 62 s | 1 (the hidden cube on the first leg) |
| wasm vs native plans on the 200 queries from `plan_report` | 200 of 200 same waypoint count, path lengths identical |

The worst replan runs inside one physics step, so it holds up that frame. Natively the worst one in `results/lidar.md` is 145 ms.
