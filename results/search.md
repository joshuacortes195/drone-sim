# Search coverage

Measured 2026-11-06 on my Intel Mac with `tools/search_report.cpp`. It flies the page's lawnmower search through the page's own bindings (`wasm/bindings.cpp` built natively, the same way `tests/test_bindings.cpp` does it), so it's the same mission a visitor starts with the search button.

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
- whole run: 134 s for 100 calm searches, 124 s windy. Run with `./build/search_report 100` (calm) and `./build/search_report 100 4 1.5` (breeze, gusts)

## Results

In calm air after the heading fix (see "The crash this found" below), seeds 1 to 100:

| | |
|---|---|
| searches finished | 100 of 100 (no crash, nothing skipped, never stuck) |
| covered (min, median, mean) | 99.78%, 100.00%, 99.977% |
| covered, open ground only (min, median, mean) | 99.77%, 100.00%, 99.980% |
| searches under 100% | 33 |
| farthest from the planned rows (median, p95, worst) | 0.480, 0.604, 0.776 m |
| sweep time (min, median, max) | 99.5, 99.5, 102.0 s |
| flown during the sweep (min, median, max) | 407.2, 407.8, 408.8 m, against 406.0 m of rows |
| searches with a replan | 24 (22 with one, 2 with two) |

In a 4 m/s breeze with 1.5 m/s gusts, seeds 1 to 100:

| | |
|---|---|
| searches finished | 100 of 100 (no crash, nothing skipped, never stuck) |
| covered (min, median, mean) | 99.45%, 99.96%, 99.938% |
| covered, open ground only (min, median, mean) | 99.58%, 99.97%, 99.950% |
| searches under 100% | 80 |
| farthest from the planned rows (median, p95, worst) | 0.495, 0.687, 0.887 m |
| sweep time (min, median, max) | 99.5, 99.5, 101.9 s |
| flown during the sweep (min, median, max) | 406.8, 407.7, 410.4 m |
| searches with a replan | 23 (22 with one, 1 with three) |

Wind pushes the rows around a little more, so more searches lose a sliver, but the worst one still covers 99.45%.

Seed 1, the page's city: covered 99.85% (99.91% of open ground), 0.691 m at most from the rows, 102.0 s, 407.7 m, one replan. In the wind it covered 100.00%, 0.461 m at most from the rows.

The rows are 11.6 m apart and each one covers 6 m to each side, so each row has only 0.2 m to spare. That's why a drone that stays within half a meter of its rows still misses a sliver: a cell drops out only where both rows next to it bulge away at the same spot. In calm air what's left uncovered is never more than 0.22% of the square. Tighter rows would leave more room for those bulges, but I haven't measured that.

## The crash this found

The first run, before the fix, finished 99 of 100. Seed 31 crashed on the way home after a complete sweep: the home leg starts at the north west corner facing west and heads south east, so the nose has to come round 135 degrees right as the drone starts moving. The attitude loop read roll and pitch off the whole attitude error, and with that much heading still to go the roll and pitch it worked out were turned by about half of it. The drone leaned partly the wrong way, the position loop fought it, it tilted to 44 degrees, ran 1.09 m off its setpoint and sank 0.62 m, and clipped the edge of a 5.91 m roof at 6.13 m. The estimate was within about a centimeter of the real drone the whole time, so it wasn't the sensors.

The fix caps how far the heading setpoint can lead the nose at 0.32 rad, a bit past where the yaw speed tops out anyway (0.8 rad/s at a gain of 3 is 0.27 rad), so turns are just as fast but roll and pitch never get read off a big heading error. On the same seed 31 home leg: 0.42 m off the setpoint at most, 0.04 m low at most, 27.6 degrees of tilt, no crash. Test: "a 1 m step in x while turning 143 degrees flies along world x and keeps its height" in `tests/test_controller.cpp` (before the fix it slid 0.19 m sideways and sank 0.43 m, after it 0.015 m and 0.008 m). Details in `results/control.md`.

Before the fix, the 99 searches that finished:

| | before | after |
|---|---|---|
| finished | 99 of 100 (seed 31 crashed) | 100 of 100 |
| covered (min, median, mean) | 99.70%, 100.00%, 99.987% | 99.78%, 100.00%, 99.977% |
| searches under 100% | 15 | 33 |
| farthest from the rows (median, p95, worst) | 0.496, 0.700, 1.221 m | 0.480, 0.604, 0.776 m |
| searches with a replan | 13 | 24 |

The track stays closer to the rows after the fix, but more searches lose a sliver and almost twice as many replan. I haven't looked into why. My guess is the drone leans differently through the corners, so the lidar sees different things, but I haven't checked that.

## Limits

- the footprint is plain geometry: 6 m to each side, with nothing hiding anything. A building between the drone and a cell doesn't stop that cell counting. The lidar's own view isn't what's scored
- it only measures where the drone flew, not whether it would have found anything
- the windy run only tries one wind, with one gust sequence shared by every city
