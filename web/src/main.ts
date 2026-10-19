import * as THREE from 'three';
import createDrone from './wasm/drone.js';
import { buildPlot } from './plot';
import {
  buildDrone,
  buildGhost,
  buildHits,
  buildMapArrow,
  buildOldPath,
  buildPath,
  buildShadow,
  buildTarget,
  buildUnknown,
  buildWorld,
} from './scene';

// the physics is z up (x east, y north), so every three.js object is too
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

// most time one frame may spend on physics, past that the sim slows down instead of the frame rate falling apart
const PHYSICS_BUDGET_MS = 8;
// a frame longer than this (like coming back to a hidden tab) only counts this much, so the drone doesn't jump ahead
const MAX_FRAME_S = 0.1;

// which seeded city to fly in
const SEED = 1;
// demo loop over street crossings at different heights, the low ones make it weave between buildings.
// the next goal is planned once the path is done and the drone is within 1 m of its end
const GOALS: [number, number, number][] = [
  [18, 18, 4],
  [-18, 18, 12],
  [-18, -18, 3],
  [18, -6, 7],
  [0, 0, 5],
];
const ARRIVED_M = 1;

// loads the wasm physics and starts the drone hovering just off the ground at the origin
const sim = await createDrone();
const DT = sim._step_size();
sim._reset(0, 0, 1.5, 0);
// builds the city in the wasm side, the page reads its buildings back to draw them
const boxes = new Float64Array(sim.HEAPF64.buffer, sim._boxes_ptr(), 6 * sim._new_city(SEED));

// views into the two state buffers: p 0-2, v 3-5, q (w x y z) 6-9, body rates 10-12, rotors 13-16
const now = new Float64Array(sim.HEAPF64.buffer, sim._state_now(), 17);
const prev = new Float64Array(sim.HEAPF64.buffer, sim._state_prev(), 17);
// the same two buffers for the estimate the controller flies on
const estNow = new Float64Array(sim.HEAPF64.buffer, sim._est_now(), 17);
const estPrev = new Float64Array(sim.HEAPF64.buffer, sim._est_prev(), 17);
// the last lidar turn's hit points, 16 beams x 180 steps at most
const MAX_HITS = 16 * 180;
const hitBuf = new Float64Array(sim.HEAPF64.buffer, sim._hits_ptr(), 3 * MAX_HITS);

// renderer filling the window
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// the world, the drone, its ground shadow, the target marker and the map view arrow
const scene = new THREE.Scene();
buildWorld(scene, boxes);
const drone = buildDrone(sim);
const pathLine = buildPath();
const shadow = buildShadow();
const target = buildTarget();
const arrow = buildMapArrow();
const ghost = buildGhost(sim);
const hits = buildHits(MAX_HITS);
const oldPath = buildOldPath();
scene.add(drone, ghost, hits, shadow, target, arrow, pathLine, oldPath);

// the live error plot under the readout
const plot = buildPlot(document.getElementById('plot') as HTMLCanvasElement);

// two cameras: one following behind the drone, and a flat map view looking straight down with north up
const chase = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.05, 500);
const top = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.5, 500);
top.up.set(0, 1, 0);
let camera: THREE.Camera = chase;
chase.position.set(-3, 0, 2);

// meters of ground the map view shows across its narrower side, so a phone held upright sees as much as a wide screen
const TOP_VIEW_M = 60;

// keeps both cameras the right shape for the window
function fitCameras() {
  const aspect = window.innerWidth / window.innerHeight;
  chase.aspect = aspect;
  chase.updateProjectionMatrix();
  const halfH = (TOP_VIEW_M / 2) * Math.max(1, 1 / aspect);
  top.top = halfH;
  top.bottom = -halfH;
  top.left = -halfH * aspect;
  top.right = halfH * aspect;
  top.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}
fitCameras();
window.addEventListener('resize', fitCameras);

// camera switch, from the button or the c key
const camButton = document.getElementById('cam') as HTMLButtonElement;
function toggleCamera() {
  camera = camera === chase ? top : chase;
  arrow.visible = camera === top;
  camButton.textContent = camera === chase ? 'top down view (c)' : 'chase view (c)';
}
camButton.addEventListener('click', toggleCamera);
window.addEventListener('keydown', (e) => {
  if (e.key === 'c') toggleCamera();
});

// plans a path from where the drone is to goal i and starts flying it, skipping goals it can't reach.
// if none can be reached it gives up and holds, instead of searching again every frame
let leg = 0;
let planMs = 0;
let pathEnd = [0, 0, 0];
let stuck = false;
let replans = 0;

// draws the path the wasm side is flying and remembers where it ends
function drawPath() {
  const n = sim._path_len();
  const pts = new Float64Array(sim.HEAPF64.buffer, sim._path_ptr(), 3 * n);
  pathLine.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(pts), 3));
  pathEnd = [pts[3 * n - 3], pts[3 * n - 2], pts[3 * n - 1]];
}

function goTo(i: number) {
  for (let tries = 0; tries < GOALS.length; tries++, i = (i + 1) % GOALS.length) {
    const [x, y, z] = GOALS[i];
    const t0 = performance.now();
    const n = sim._fly_to(x, y, z);
    planMs = performance.now() - t0;
    if (n === 0) continue;
    drawPath();
    target.position.set(x, y, z);
    leg = i;
    return;
  }
  stuck = true;
}
goTo(leg);

// picks a spot for a 2 m cube on the middle of the first path's longest leg, like tools/lidar_report.cpp.
// the drone isn't told about it, so the lidar has to find it and it has to replan
let cube: number[] | null = null;
function hideCube() {
  const n = sim._path_len();
  const pts = new Float64Array(sim.HEAPF64.buffer, sim._path_ptr(), 3 * n);
  let best = 0, bestI = 0;
  for (let i = 1; i < n; i++) {
    const d = Math.hypot(pts[3 * i] - pts[3 * i - 3], pts[3 * i + 1] - pts[3 * i - 2], pts[3 * i + 2] - pts[3 * i - 1]);
    if (d > best) [best, bestI] = [d, i];
  }
  // too short a leg would put the cube on the start or the goal
  if (best <= 6) return;
  const mid = [0, 1, 2].map((k) => (pts[3 * bestI + k] + pts[3 * bestI - 3 + k]) / 2);
  cube = [mid[0] - 1, mid[1] - 1, mid[2] - 1, mid[0] + 1, mid[1] + 1, mid[2] + 1];
}
if (!stuck) hideCube();

// drops the cube in once the drone is under way (2 m/s). natively it's there from the start, but the lidar
// would spot it from the pad and the replan would be over before anyone saw it
function dropCube() {
  if (!cube || Math.hypot(now[3], now[4], now[5]) < 2) return;
  const [x0, y0, z0, x1, y1, z1] = cube;
  sim._add_box(x0, y0, z0, x1, y1, z1);
  scene.add(buildUnknown(x0, y0, z0, x1, y1, z1));
  cube = null;
}

// moves on to the next goal once the path is flown and the drone is close enough to its end
// plain squared distance instead of Math.hypot, whose rounding can differ between browsers
function updateMission() {
  // the lidar made it replan: keep the old path faded for 2 s and draw the new one
  if (sim._replan_count() !== replans) {
    replans = sim._replan_count();
    oldPath.geometry.dispose();
    oldPath.geometry = pathLine.geometry.clone();
    oldPath.visible = true;
    oldPathUntil = sim._sim_tick() + 2000;
    drawPath();
  }
  // a hold after a failed replan is one point, so it counts as done right away and the loop moves to the next goal.
  // natively (lidar_report) the drone just stays holding
  if (stuck || !sim._path_done()) return;
  const dx = now[0] - pathEnd[0], dy = now[1] - pathEnd[1], dz = now[2] - pathEnd[2];
  if (dx * dx + dy * dy + dz * dz < ARRIVED_M * ARRIVED_M) goTo((leg + 1) % GOALS.length);
}

// draws the drone part way between the last two physics states, alpha is how far past the older one we are
const qa = new THREE.Quaternion(), qb = new THREE.Quaternion();
function placeDrone(alpha: number) {
  drone.position.set(
    prev[0] + (now[0] - prev[0]) * alpha,
    prev[1] + (now[1] - prev[1]) * alpha,
    prev[2] + (now[2] - prev[2]) * alpha,
  );
  // three.js takes quaternions as x y z w
  qa.set(prev[7], prev[8], prev[9], prev[6]);
  qb.set(now[7], now[8], now[9], now[6]);
  drone.quaternion.slerpQuaternions(qa, qb, alpha);
  shadow.position.set(drone.position.x, drone.position.y, 0.02);

  // the ghost the same way, between the last two estimates
  ghost.position.set(
    estPrev[0] + (estNow[0] - estPrev[0]) * alpha,
    estPrev[1] + (estNow[1] - estPrev[1]) * alpha,
    estPrev[2] + (estNow[2] - estPrev[2]) * alpha,
  );
  qa.set(estPrev[7], estPrev[8], estPrev[9], estPrev[6]);
  qb.set(estNow[7], estNow[8], estNow[9], estNow[6]);
  ghost.quaternion.slerpQuaternions(qa, qb, alpha);
}

// copies the last lidar turn's hits into the dots, only when a new turn came in.
// turns happen on step 50 of every 100, so this counts them off the wasm clock
let hitsShown = -1;
function placeHits() {
  const scans = Math.floor((sim._sim_tick() + 50) / 100);
  if (scans === hitsShown) return;
  hitsShown = scans;
  const n = sim._hits_len();
  const pos = hits.geometry.getAttribute('position') as THREE.BufferAttribute;
  for (let i = 0; i < 3 * n; i++) pos.array[i] = hitBuf[i];
  pos.needsUpdate = true;
  hits.geometry.setDrawRange(0, n);
}

// the plot gets the average error over each 100 steps (10 a second of sim time). a single sample would always land
// on the gps fix step, right when the error is smallest
let errSum = 0;
let oldPathUntil = 0;
function afterStep() {
  const tick = sim._sim_tick();
  errSum += Math.hypot(now[0] - estNow[0], now[1] - estNow[1], now[2] - estNow[2]);
  if (tick % 100 === 0) {
    plot.push(errSum / 100);
    errSum = 0;
  }
  // the old path fades out 2 s after a replan
  if (oldPath.visible && tick >= oldPathUntil) oldPath.visible = false;
  dropCube();
}

// chase camera eases toward a spot 3 m behind and 1 m above the drone, along its heading
const forward = new THREE.Vector3();
const want = new THREE.Vector3();
function placeCameras(frameS: number) {
  // the drone's nose direction flattened onto the ground
  forward.set(1, 0, 0).applyQuaternion(drone.quaternion);
  forward.z = 0;
  if (forward.lengthSq() < 1e-6) forward.set(1, 0, 0);
  forward.normalize();
  want.copy(drone.position).addScaledVector(forward, -3);
  want.z += 1;
  // exponential easing, the same feel at any frame rate. at 8 m/s it trails about 1 m extra
  chase.position.lerp(want, 1 - Math.exp(-8 * frameS));
  chase.lookAt(drone.position);

  // map view sits straight above the drone, above every building but close enough that the fog doesn't wash it out
  top.position.set(drone.position.x, drone.position.y, 40);

  // the map arrow sits on top of the drone turned to its heading
  arrow.position.set(drone.position.x, drone.position.y, drone.position.z + 0.5);
  arrow.rotation.z = Math.atan2(forward.y, forward.x);
  top.lookAt(drone.position);
}

// numbers for the readout, reset every half second
const hud = document.getElementById('hud') as HTMLElement;
let frames = 0;
let stepsDone = 0;
let physicsMs = 0;
let windowStart = performance.now();

// real time that has passed but hasn't been simulated yet
let backlog = 0;
let last = performance.now();

// runs once per screen refresh: catch the physics up in fixed 1 ms steps, then draw in between the last two
function frame(t: number) {
  const frameS = Math.min(MAX_FRAME_S, Math.max(0, t - last) / 1000);
  backlog += frameS;
  last = t;

  // as many whole steps as the backlog covers
  // floored at 0 because the rounding can leave a tiny negative leftover
  const steps = Math.floor(backlog / DT);
  backlog = Math.max(0, backlog - steps * DT);

  // one step at a time until done or out of time, then drop what's left so a slow frame can't snowball
  // checking the mission after every step means waypoints switch on the same step on any machine
  if (steps > 0) {
    const t0 = performance.now();
    let done = 0;
    while (done < steps && performance.now() - t0 < PHYSICS_BUDGET_MS) {
      sim._advance(1);
      afterStep();
      updateMission();
      done++;
    }
    if (done < steps) backlog = 0;
    physicsMs += performance.now() - t0;
    stepsDone += done;
  }

  placeDrone(backlog / DT);
  placeHits();
  placeCameras(frameS);
  renderer.render(scene, camera);

  // fresh readout twice a second
  frames++;
  if (t - windowStart >= 500) {
    const seconds = (t - windowStart) / 1000;
    const speed = Math.hypot(now[3], now[4], now[5]);
    hud.textContent =
      `${(frames / seconds).toFixed(0)} fps · ${(stepsDone / seconds).toFixed(0)} steps/s · ` +
      `${(physicsMs / frames).toFixed(2)} ms physics/frame · ${speed.toFixed(1)} m/s · alt ${now[2].toFixed(1)} m · ` +
      `${replans} replan${replans === 1 ? '' : 's'}`;
    frames = 0;
    stepsDone = 0;
    physicsMs = 0;
    windowStart = t;
  }

  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
