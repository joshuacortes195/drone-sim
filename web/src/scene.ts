import * as THREE from 'three';
import type { Drone } from './wasm/drone';

// a box sitting on the ground: center x and y, size x and y, height, all in meters
export type Box = [cx: number, cy: number, sx: number, sy: number, h: number];

// placeholder buildings around the demo route until the seeded city map lands in phase 4
export const BOXES: Box[] = [
  [12.5, 12.5, 10, 10, 15],
  [5, 19, 3, 3, 6],
  [-8, 10, 6, 10, 12],
  [35, 8, 8, 6, 9],
  [14, 35, 10, 6, 16],
  [32, 32, 6, 6, 6],
  [-6, -8, 5, 5, 7],
  [12, -9, 8, 5, 11],
];

// sky, light, ground grid and the buildings
export function buildWorld(scene: THREE.Scene) {
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
  const grid = new THREE.GridHelper(200, 100, 0x7c8570, 0x939c86);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = 0.01;
  scene.add(grid);

  // one box mesh per building, lifted so it sits on the ground
  const boxMat = new THREE.MeshLambertMaterial({ color: 0x8796a8 });
  for (const [cx, cy, sx, sy, h] of BOXES) {
    const box = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, h), boxMat);
    box.position.set(cx, cy, h / 2);
    scene.add(box);
  }
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
