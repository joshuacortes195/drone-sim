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

The readout in the corner averages over half a second. I sampled it every 500 ms for 30 s (58 distinct windows) on the chase camera:

| | min | median | max |
|---|---|---|---|
| frames per second | 60 | 60 | 60 |
| physics steps per second | 999 | 1000 | 1002 |
| physics time per frame (ms, coarse timer) | 0.02 | 0.04 | 0.09 |

60 fps is the display's refresh rate, and every half second window read 60 (the readout rounds to whole frames). The physics kept real time: 1000 steps of 1 ms each per second. No console errors or page errors on a fresh load. (Rebuilding the wasm while the dev server is open does throw a few errors: Vite reloads the page while Emscripten is part way through rewriting `drone.js` and `drone.wasm`, so for a moment they don't match. A reload after the build fixes it.)

## Cost of one step in the browser

One `advance(100000)` call (controller plus RK4 physics, 100,000 steps of 1 ms) timed in the page, from hovering at 10 m with a target 22.4 m away, five runs:

| run | µs per step |
|---|---|
| 1 | 0.671 |
| 2 | 0.680 |
| 3 | 0.661 |
| 4 | 0.680 |
| 5 | 0.670 |

So 1000 steps a second cost about 0.67 ms of CPU per real second, well under the 8 ms budget. Natively the two parts were measured separately: 308 to 394 ns per RK4 step (`results/physics.md`) and 268 to 300 ns per `control()` call (`results/control.md`).

## Download size

From `npm run build`:

| file | size | gzip |
|---|---|---|
| `drone.wasm` | 19.89 kB | 10.52 kB |
| page js (three.js plus my code) | 564.21 kB | 142.66 kB |
| `index.html` | 0.87 kB | 0.53 kB |
