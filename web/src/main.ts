import * as THREE from 'three';
import createDrone from './wasm/drone.js';
import { buildDrone, buildMapArrow, buildShadow, buildTarget, buildWorld } from './scene';

// the physics is z up (x east, y north), so every three.js object is too
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

// most time one frame may spend on physics, past that the sim slows down instead of the frame rate falling apart
const PHYSICS_BUDGET_MS = 8;
// a frame longer than this (like coming back to a hidden tab) only counts this much, so the drone doesn't jump ahead
const MAX_FRAME_S = 0.1;

// demo loop around the placeholder buildings at 8 m, the next point is sent once the drone gets within 1 m
const ROUTE: [number, number, number][] = [
  [0, 0, 8],
  [25, 0, 8],
  [25, 25, 8],
  [0, 25, 8],
];
const ARRIVED_M = 1;

// loads the wasm physics and starts the drone hovering just off the ground at the origin
const sim = await createDrone();
const DT = sim._step_size();
sim._reset(0, 0, 1.5, 0);

// views into the two state buffers: p 0-2, v 3-5, q (w x y z) 6-9, body rates 10-12, rotors 13-16
const now = new Float64Array(sim.HEAPF64.buffer, sim._state_now(), 17);
const prev = new Float64Array(sim.HEAPF64.buffer, sim._state_prev(), 17);

// renderer filling the window
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// the world, the drone, its ground shadow, the target marker and the map view arrow
const scene = new THREE.Scene();
buildWorld(scene);
const drone = buildDrone(sim);
const shadow = buildShadow();
const target = buildTarget();
const arrow = buildMapArrow();
scene.add(drone, shadow, target, arrow);

// two cameras: one following behind the drone, and a flat map view looking straight down with north up
const chase = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.05, 500);
const top = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.5, 500);
top.up.set(0, 1, 0);
let camera: THREE.Camera = chase;
chase.position.set(-3, 0, 2);

// meters of ground the map view shows top to bottom
const TOP_VIEW_M = 60;

// keeps both cameras the right shape for the window
function fitCameras() {
  const aspect = window.innerWidth / window.innerHeight;
  chase.aspect = aspect;
  chase.updateProjectionMatrix();
  top.top = TOP_VIEW_M / 2;
  top.bottom = -TOP_VIEW_M / 2;
  top.left = (-TOP_VIEW_M / 2) * aspect;
  top.right = (TOP_VIEW_M / 2) * aspect;
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

// sends the drone to waypoint i, facing from where it is now toward it (straight up counts as facing east)
let leg = 0;
function sendTo(i: number) {
  const [x, y, z] = ROUTE[i];
  const yaw = Math.atan2(y - now[1], x - now[0]);
  sim._set_target(x, y, z, yaw);
  target.position.set(x, y, z);
}
sendTo(leg);

// moves on to the next waypoint once the drone is close enough to this one
// plain squared distance instead of Math.hypot, whose rounding can differ between browsers
function updateMission() {
  const [x, y, z] = ROUTE[leg];
  const dx = now[0] - x, dy = now[1] - y, dz = now[2] - z;
  if (dx * dx + dy * dy + dz * dz < ARRIVED_M * ARRIVED_M) {
    leg = (leg + 1) % ROUTE.length;
    sendTo(leg);
  }
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
      updateMission();
      done++;
    }
    if (done < steps) backlog = 0;
    physicsMs += performance.now() - t0;
    stepsDone += done;
  }

  placeDrone(backlog / DT);
  placeCameras(frameS);
  renderer.render(scene, camera);

  // fresh readout twice a second
  frames++;
  if (t - windowStart >= 500) {
    const seconds = (t - windowStart) / 1000;
    const speed = Math.hypot(now[3], now[4], now[5]);
    hud.textContent =
      `${(frames / seconds).toFixed(0)} fps · ${(stepsDone / seconds).toFixed(0)} steps/s · ` +
      `${(physicsMs / frames).toFixed(2)} ms physics/frame · ${speed.toFixed(1)} m/s · alt ${now[2].toFixed(1)} m`;
    frames = 0;
    stepsDone = 0;
    physicsMs = 0;
    windowStart = t;
  }

  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
