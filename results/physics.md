# Physics results (phase 1)

Measured 2026-10-14 with `tools/physics_report.cpp`.

## How

- machine: Intel Core i5-8259U @ 2.30GHz, 8 GB RAM, macOS 15.7.7
- compiler: Apple clang 17.0.0, CMake Release build (`-O3 -DNDEBUG`), C++17, double precision
- airframe: RotorS Iris parameters from `docs/plan.md` (mass 1.52 kg, body inertia only)
- timestep: 1 ms fixed
- run it: `cmake --build build && ./build/physics_report`

## Free fall

Motors off, starting at rest at 100 m, RK4 for 4 s (4000 steps), compared against z0 - g t² / 2.

| | error after 4 s |
|---|---|
| height | -8.2e-12 m |
| vertical speed | -5.2e-12 m/s |

That's just rounding. Any second order method is exact under constant acceleration, so this checks the wiring, not the integrator.

## RK4 vs semi-implicit Euler on a torque free tumble

Motors off, starting body rate ω = (1, 2, 3) rad/s (not about a principal axis, so the ω × Iω term is working the whole time), 60 s (60,000 steps) at 1 ms. With no torque, the world frame angular momentum R I ω and the rotational energy ½ ωᵀ I ω should never change. The table shows the worst relative drift seen at any step.

| integrator | worst momentum drift | worst energy drift |
|---|---|---|
| RK4 | 2.1e-12 | 1.0e-12 |
| semi-implicit Euler (rates first, then position and quaternion with the new rates) | 8.8e-2 (8.8%) | 3.9e-1 (39%) |

Decision: RK4. Euler loses more than a third of the spin energy in a minute. RK4 holds it to rounding error.

## Cost of one step

One RK4 step at hover (4 derivative evaluations plus renormalizing), averaged over 1,000,000 steps: 313 to 394 ns across three runs on the machine above. One second of flight at 1 ms steps is 1000 steps, so well under a millisecond of CPU per simulated second natively. The browser number gets measured separately in phase 3.
