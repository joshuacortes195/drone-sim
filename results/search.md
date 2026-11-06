# Search coverage

Measured 2026-11-06 on my Intel Mac with `tools/search_report.cpp`. It flies the page's lawnmower search through the page's own bindings (`wasm/bindings.cpp` built natively, the same way `tests/test_bindings.cpp` does it), so it's the same mission a visitor starts with the search button.

Rerun the same day after braking replans (2026-11-06, commits 6f65b20 and d1c7dcb, the second one the review fixes). A replan at speed used to start the new path from rest, so the setpoint stopped dead and the drone overshot it. Now it plans from where the drone would stop braking straight ahead at 5 m/s² and brakes into the new path from its current velocity, falling back to the old way when that braking line would come within the planner's 0.79 m margin of something known. The tables in "Results" are that run, with the values from after the heading fix in brackets where they moved. Only searches that replanned changed, 2 calm and 6 windy, and none of them lost coverage. In calm air seed 22 replans once instead of twice and seed 86 went from 99.91% to 100.00%. In the wind seed 37, the worst one before, replans once instead of three times and went from 99.45% to 100.00%, its worst stray from 0.887 to 0.495 m, and seeds 28 and 57 got most of their slivers back too. Seed 1 in the wind strays a bit more (0.461 to 0.511 m) and still covers 100.00%. A first braking run before the review fixes (a looser check on the braking line) changed more searches and made some worse: in the wind seed 6 dropped from 99.98% to 99.54%, and in calm air seed 12 replanned 4 times and had the longest sweep, 103.1 s.

## How

- machine: Intel Mac, Core i5-8259U @ 2.30GHz (4 cores, 8 threads), 8 GB RAM, macOS 15.7.7
- build: Apple clang 17.0.0, CMake Release build (`-O3 -DNDEBUG`), C++17, double precision, one thread
- maps: seeds 1 to 100, one city each from `city()` in `core/map.cpp` (5 x 5 lots, 8 m lots on a 12 m grid, buildings 4 to 18 m tall, 20% of lots empty). Seed 1 is the city the page flies
- mission: the page's search. It starts hovering 1.5 m over home like the page does after a load, then flies `lawnmower(-29, -29, 29, 29, 6, 12)` (six rows 11.6 m apart along the east west streets, 6 m up, 406.0 m of rows end to end) and then home, 2 m over the pad. Each leg goes through A* like a tap does, and the next one starts once the path is flown and the drone is within 1 m of its end, same as the page
- flying: the full stack, the same as the page. Physics on a fixed 1 ms RK4 step, the cascaded PID controller with the gains in `core/controller.h`, flying on the RTK estimate, the VLP-16 lidar at 10 Hz with replanning, 5 m/s cruise and 3 m/s² ramps
- the track: the real drone's position every 10 ms, from reaching the start of the first row to reaching the end of the last one. The climb to the first row and the trip home don't count
- coverage: the 58 x 58 m search square cut into 0.25 m cells. A cell counts as searched if the track passed within 6 m of its center sideways, half the 12 m row spacing, which is what the spacing assumes each side of a row can see. "Open ground" leaves out cells under a building, where something on the ground couldn't be seen anyway
- cross check: the planned rows themselves, walked every 0.1 m, cover 100.00% of the square, so the cell counting does what it should
- wind: the calm run has none. The windy run uses the eval's middle setting, a 4 m/s breeze from the west plus 1.5 m/s gusts, set the way the page's sliders set it. Like the page, every reset starts the gusts calm from the same gust seed, so every city gets the same gust sequence
- whole run: 134 s for 100 calm searches, 124 s windy (timed before braking replans, the rerun ran next to other runs so its times aren't used). Run with `./build/search_report 100` (calm) and `./build/search_report 100 4 1.5` (breeze, gusts)

## Results

In calm air after braking replans (2026-11-06, commit d1c7dcb), seeds 1 to 100. Brackets hold the values from after the heading fix (see "The crash this found" below) where they moved:

| | |
|---|---|
| searches finished | 100 of 100 (no crash, nothing skipped, never stuck) |
| covered (min, median, mean) | 99.78%, 100.00%, 99.977% |
| covered, open ground only (min, median, mean) | 99.77%, 100.00%, 99.981% (99.980%) |
| searches under 100% | 32 (33) |
| farthest from the planned rows (median, p95, worst) | 0.478 (0.480), 0.604, 0.776 m |
| sweep time (min, median, max) | 99.5, 99.5, 102.0 s |
| flown during the sweep (min, median, max) | 407.2, 407.8, 408.8 m, against 406.0 m of rows |
| searches with a replan, whole mission (climb and home legs too) | 24 (23 with one, 1 with two; before: 22 with one, 2 with two) |

In a 4 m/s breeze with 1.5 m/s gusts after braking replans, seeds 1 to 100, brackets the same way:

| | |
|---|---|
| searches finished | 100 of 100 (no crash, nothing skipped, never stuck) |
| covered (min, median, mean) | 99.52% (99.45%), 99.96%, 99.948% (99.938%) |
| covered, open ground only (min, median, mean) | 99.76% (99.58%), 99.98% (99.97%), 99.958% (99.950%) |
| searches under 100% | 79 (80) |
| farthest from the planned rows (median, p95, worst) | 0.495, 0.551 (0.687), 0.724 (0.887) m |
| sweep time (min, median, max) | 99.5, 99.5, 101.9 s |
| flown during the sweep (min, median, max) | 406.8, 407.7, 410.4 m |
| searches with a replan, whole mission (climb and home legs too) | 23 (all with one; before: 22 with one, 1 with three) |

Wind pushes the rows around a little more, so more searches lose a sliver, but the worst one still covers 99.52% (seed 84, which doesn't replan and didn't change; before braking replans the worst was seed 37 at 99.45%).

Seed 1, the page's city: covered 99.85% (99.91% of open ground), 0.691 m at most from the rows, 102.0 s, 407.7 m, one replan, the same as before braking replans. In the wind it covered 100.00%, 0.511 m at most from the rows, 101.4 s, 408.4 m (0.461 m, 100.3 s and 407.2 m before).

The rows are 11.6 m apart and each one covers 6 m to each side, so neighbouring rows overlap by only 0.4 m. If one row drifts more than 0.4 m away from its neighbour somewhere, a sliver between them drops out. Every search strays more than 0.4 m somewhere (the least is 0.410 m), and half the calm ones stray 0.478 m or more at their worst point, but a drift only opens a gap when it's away from the neighbouring row, so about a third of them (32) end up missing a little. In calm air what's left uncovered is never more than 0.22% of the square. Tighter rows would leave more room for that drift, but I haven't measured it.

## The crash this found

The first run, before the fix, finished 99 of 100. Seed 31 crashed on the way home after a complete sweep: the home leg starts at the north west corner facing west and heads south east, so the nose has to come round 135 degrees right as the drone starts moving. The attitude loop read roll and pitch off the whole attitude error, and with that much heading still to go the roll and pitch it worked out were turned by about half of it. The drone leaned partly the wrong way, the position loop fought it, it tilted to 44 degrees, ran 1.09 m off its setpoint and sank 0.62 m, and clipped the edge of a 5.91 m roof at 6.13 m. The estimate was within about a centimeter of the real drone the whole time, so it wasn't the sensors.

The fix caps how far the heading setpoint can lead the nose at 0.32 rad, a bit past where the yaw speed tops out anyway (0.8 rad/s at a gain of 3 is 0.27 rad), so roll and pitch never get read off a big heading error. A turn on the spot takes as long as before, a turn while moving up to 0.73 s longer (`results/control.md`, "Heading cap"). On the same seed 31 home leg: 0.42 m off the setpoint at most, 0.04 m low at most, 27.6 degrees of tilt, no crash. Test: "a 1 m step in x while turning 143 degrees flies along world x and keeps its height" in `tests/test_controller.cpp` (before the fix it slid 0.19 m sideways and sank 0.43 m, after it 0.015 m and 0.008 m). Details in `results/control.md`.

Before the fix, the 99 searches that finished:

| | before | after (before braking replans) |
|---|---|---|
| finished | 99 of 100 (seed 31 crashed) | 100 of 100 |
| covered (min, median, mean) | 99.70%, 100.00%, 99.987% | 99.78%, 100.00%, 99.977% |
| searches under 100% | 15 | 33 |
| farthest from the rows (median, p95, worst) | 0.496, 0.700, 1.221 m | 0.480, 0.604, 0.776 m |
| searches with a replan, whole mission | 13 | 24 |

The track stays closer to the rows after the fix, but more searches lose a sliver and almost twice as many replan. I haven't looked into why. My guess is the drone leans differently through the corners, so the lidar sees different things, but I haven't checked that.

## Limits

- the footprint is plain geometry: 6 m to each side, with nothing hiding anything. A building between the drone and a cell doesn't stop that cell counting. The lidar's own view isn't what's scored
- it only measures where the drone flew, not whether it would have found anything
- the windy run only tries one wind, with one gust sequence shared by every city
