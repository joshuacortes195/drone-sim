import * as THREE from 'three';
import type { Drone } from './wasm/drone';

// sky, light, ground grid and the buildings. boxes holds 6 numbers per building: low x y z, high x y z.
// hands back the ground and buildings so a tap can find what it landed on
export function buildWorld(scene: THREE.Scene, boxes: Float64Array) {
  const pickable: THREE.Mesh[] = [];
  // pale sky with fog so far away things fade out
  scene.background = new THREE.Color(0xcfd8e3);
  scene.fog = new THREE.Fog(0xcfd8e3, 60, 180);

  // soft light from the sky plus a sun so box faces read differently
  scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f80, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.4);
  sun.position.set(30, -20, 50);
  scene.add(sun);

  // flat ground with a 2 m grid on it, the grid helper is built flat in xz so it gets turned to lie in xy
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(400, 400), new THREE.MeshLambertMaterial({ color: 0xa9b29a }));
  scene.add(ground);
  pickable.push(ground);
  const grid = new THREE.GridHelper(200, 100, 0x7c8570, 0x939c86);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = 0.01;
  scene.add(grid);

  // one box mesh per building, lifted so it sits on the ground
  const boxMat = new THREE.MeshLambertMaterial({ color: 0x8796a8 });
  for (let i = 0; i < boxes.length; i += 6) {
    const [x0, y0, z0, x1, y1, z1] = boxes.subarray(i, i + 6);
    const box = new THREE.Mesh(new THREE.BoxGeometry(x1 - x0, y1 - y0, z1 - z0), boxMat);
    box.position.set((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2);
    scene.add(box);
    pickable.push(box);
  }
  return pickable;
}

// the quad: a body, an arm out to each rotor and a disc for each rotor, all at the physics' rotor positions
export function buildDrone(sim: Drone) {
  const drone = new THREE.Group();

  // dark body in the middle
  const dark = new THREE.MeshLambertMaterial({ color: 0x2b2f36 });
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.12, 0.05), dark);
  drone.add(body);

  // front rotors orange and back ones gray, so you can tell which way it faces
  const front = new THREE.MeshBasicMaterial({ color: 0xff8a2a, transparent: true, opacity: 0.75, side: THREE.DoubleSide });
  const back = new THREE.MeshBasicMaterial({ color: 0x9aa3ad, transparent: true, opacity: 0.75, side: THREE.DoubleSide });
  for (let i = 0; i < 4; i++) {
    const x = sim._rotor_pos(i, 0), y = sim._rotor_pos(i, 1), z = sim._rotor_pos(i, 2);

    // thin arm from the center out to the rotor, turned to point at it
    const len = Math.hypot(x, y);
    const arm = new THREE.Mesh(new THREE.BoxGeometry(len, 0.02, 0.02), dark);
    arm.position.set(x / 2, y / 2, z / 2);
    arm.rotation.z = Math.atan2(y, x);
    drone.add(arm);

    // the spinning rotor drawn as a see-through disc with the 0.1 m rotor radius
    const disc = new THREE.Mesh(new THREE.CircleGeometry(0.1, 24), x > 0 ? front : back);
    disc.position.set(x, y, z + 0.005);
    drone.add(disc);
  }
  return drone;
}

// a soft dark spot on the ground under the drone so you can judge its height
export function buildShadow() {
  const mat = new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.25, depthWrite: false });
  const shadow = new THREE.Mesh(new THREE.CircleGeometry(0.35, 24), mat);
  shadow.position.z = 0.02;
  return shadow;
}

// the planned path as a line, its points get swapped in each time the drone plans
export function buildPath() {
  const line = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x2a9d8f }));
  line.frustumCulled = false;
  return line;
}

// the path it just gave up after a replan, faded, so you can see what changed
export function buildOldPath() {
  const mat = new THREE.LineBasicMaterial({ color: 0x2a9d8f, transparent: true, opacity: 0.3, depthWrite: false });
  const line = new THREE.Line(new THREE.BufferGeometry(), mat);
  line.frustumCulled = false;
  line.visible = false;
  return line;
}

// small marker where the drone is being sent
export function buildTarget() {
  return new THREE.Mesh(new THREE.SphereGeometry(0.15, 16, 12), new THREE.MeshBasicMaterial({ color: 0x2a9d8f }));
}

// a 2 m arrow pointing where the drone faces, only shown in the map view where the drone itself is a few pixels
export function buildMapArrow() {
  const shape = new THREE.Shape();
  shape.moveTo(1.2, 0);
  shape.lineTo(-0.8, 0.7);
  shape.lineTo(-0.4, 0);
  shape.lineTo(-0.8, -0.7);
  shape.closePath();
  const arrow = new THREE.Mesh(new THREE.ShapeGeometry(shape), new THREE.MeshBasicMaterial({ color: 0xff8a2a }));
  arrow.visible = false;
  return arrow;
}

// colors shared with the error plot and the key, checked as a set for color blind contrast
export const COLORS = { path: 0x2a9d8f, estimate: 0x4a6fe3, hits: 0xd6336c };

// the ghost: the same quad drawn see-through in blue where the estimate says the drone is
export function buildGhost(sim: Drone) {
  const ghost = buildDrone(sim);
  const mat = new THREE.MeshBasicMaterial({ color: COLORS.estimate, transparent: true, opacity: 0.45, depthWrite: false });
  ghost.traverse((o) => {
    if (o instanceof THREE.Mesh) o.material = mat;
  });
  // a bit bigger so it shows around the real drone when the two sit on top of each other
  ghost.scale.setScalar(1.15);
  return ghost;
}

// the last lidar turn's hits as dots, room for every beam of one turn
export function buildHits(max: number) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * max), 3));
  geo.setDrawRange(0, 0);
  const pts = new THREE.Points(geo, new THREE.PointsMaterial({ color: COLORS.hits, size: 3, sizeAttenuation: false }));
  pts.frustumCulled = false;
  return pts;
}

// a box the drone wasn't told about: faint fill with dark edges, so it reads as something other than the city
export function buildUnknown(x0: number, y0: number, z0: number, x1: number, y1: number, z1: number) {
  const geo = new THREE.BoxGeometry(x1 - x0, y1 - y0, z1 - z0);
  const box = new THREE.Mesh(geo, new THREE.MeshLambertMaterial({ color: 0x8796a8, transparent: true, opacity: 0.35 }));
  box.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo), new THREE.LineBasicMaterial({ color: 0x1d2733 })));
  box.position.set((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2);
  return box;
}
