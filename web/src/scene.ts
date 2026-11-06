import * as THREE from 'three';
import { Sky } from 'three/addons/objects/Sky.js';
import type { Drone } from './wasm/drone';
import { CITY_M, STYLES, cityGround, facade, grass, roof } from './textures';

// where the sun sits: a little west of south and 50 deg up, so buildings throw shadows you can read without the
// streets going black
const SUN = new THREE.Vector3(-0.35, -0.55, 0.76).normalize();

// the sky: three's physical sky shader, which works in y up, so its view direction gets turned to our z up world
function buildSky() {
  const sky = new Sky();
  const u = sky.material.uniforms;
  u.turbidity.value = 3;
  u.rayleigh.value = 1.2;
  u.mieCoefficient.value = 0.004;
  u.mieDirectionalG.value = 0.8;
  u.cloudCoverage.value = 0.35;
  u.cloudDensity.value = 0.5;
  // the sun in the shader's y up axes: x stays east, up becomes y, north becomes -z
  u.sunPosition.value.set(SUN.x, SUN.z, -SUN.y);
  // swaps the look direction from z up to y up before the shader works out the sky color
  sky.material.fragmentShader = sky.material.fragmentShader.replace(
    'vec3 direction = normalize( vWorldPosition - cameraPosition );',
    'vec3 direction = normalize( vWorldPosition - cameraPosition ).xzy * vec3( 1.0, 1.0, -1.0 );',
  );
  return sky;
}

// sky, sun, ground and the buildings. boxes holds 6 numbers per building: low x y z, high x y z.
// hands back the ground and buildings so a tap can find what it landed on
export function buildWorld(scene: THREE.Scene, renderer: THREE.WebGLRenderer, boxes: Float64Array, phone: boolean) {
  const pickable: THREE.Mesh[] = [];

  // the sky gets rendered once into a blurred light map, which lights every surface softly from all sides
  // (image based lighting), then the same sky is the background
  const sky = buildSky();
  const skyScene = new THREE.Scene();
  sky.scale.setScalar(50);
  skyScene.add(sky);
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(skyScene, 0.02).texture;
  scene.environmentIntensity = 0.45;
  pmrem.dispose();
  sky.scale.setScalar(400);
  scene.add(sky);
  // haze the color of the sky near the horizon, so far ground melts into it
  scene.fog = new THREE.FogExp2(0xc9d4de, 0.009);

  // the sun, casting soft shadows over the whole city from one shadow map
  const sun = new THREE.DirectionalLight(0xfff1dc, 4);
  sun.position.copy(SUN).multiplyScalar(100);
  sun.castShadow = true;
  sun.shadow.mapSize.setScalar(phone ? 2048 : 4096);
  const cam = sun.shadow.camera;
  cam.left = cam.bottom = -42;
  cam.right = cam.top = 42;
  cam.near = 20;
  cam.far = 200;
  // a few texels of blur on the shadow edges, and a nudge that stops surfaces shadowing themselves in stripes
  sun.shadow.radius = 3;
  sun.shadow.normalBias = 0.03;
  scene.add(sun, sun.target);

  // the painted city floor in the middle, sitting on grass that runs out to the haze
  const city = new THREE.Mesh(
    new THREE.PlaneGeometry(CITY_M, CITY_M),
    new THREE.MeshStandardMaterial({ map: cityGround(boxes), roughness: 0.92 }),
  );
  city.receiveShadow = true;
  const grassTex = grass();
  grassTex.repeat.set(225, 225);
  // pushed back in depth so it never flickers through the city floor where the two overlap
  const outer = new THREE.Mesh(
    new THREE.PlaneGeometry(900, 900),
    new THREE.MeshStandardMaterial({ map: grassTex, roughness: 1, polygonOffset: true, polygonOffsetFactor: 2, polygonOffsetUnits: 2 }),
  );
  outer.receiveShadow = true;
  scene.add(outer, city);
  pickable.push(city, outer);

  // one set of materials per building look: the facade on the four sides, gravel on top and bottom
  const roofTex = roof();
  const roofMat = new THREE.MeshStandardMaterial({ map: roofTex, roughness: 0.95 });
  const looks = STYLES.map((st, k) => {
    const { map, rm } = facade(st, 100 + k);
    const wall = new THREE.MeshStandardMaterial({ map, roughnessMap: rm, metalnessMap: rm, roughness: 1, metalness: 1 });
    return [wall, wall, wall, wall, roofMat, roofMat];
  });

  // one box per building, lifted so it sits on the ground, with its picture laid on in meters so windows are 3 m apart
  // on every building whatever its size
  for (let i = 0; i < boxes.length; i += 6) {
    const [x0, y0, z0, x1, y1, z1] = boxes.subarray(i, i + 6);
    const geo = new THREE.BoxGeometry(x1 - x0, y1 - y0, z1 - z0);
    meterUVs(geo, x1 - x0, y1 - y0, z1 - z0);
    // tall ones lean to glass, the rest are picked from the index so the same city always looks the same
    const h = z1 - z0;
    const look = h > 13 ? (i / 6) % 2 === 0 ? 0 : 1 : 1 + ((i / 6) % 2);
    const box = new THREE.Mesh(geo, looks[look]);
    box.position.set((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2);
    box.castShadow = box.receiveShadow = true;
    scene.add(box);
    pickable.push(box);
  }
  return pickable;
}

// rewrites a box's texture spots in meters from its own corner. walls use 6 m per picture (2 bays, 2 floors), lined up
// so the roof lands on a floor line, roofs use 4 m per tile
function meterUVs(geo: THREE.BoxGeometry, sx: number, sy: number, sz: number) {
  const pos = geo.getAttribute('position'), nrm = geo.getAttribute('normal'), uv = geo.getAttribute('uv');
  // whatever is left over from whole 3 m floors ends up at the bottom, so the roof lands on a floor line
  const lobby = sz % 3;
  for (let k = 0; k < pos.count; k++) {
    const x = pos.getX(k) + sx / 2, y = pos.getY(k) + sy / 2, z = pos.getZ(k) + sz / 2;
    const nx = nrm.getX(k), ny = nrm.getY(k), nz = nrm.getZ(k);
    if (Math.abs(nz) > 0.5) uv.setXY(k, x / 4, y / 4);
    else if (Math.abs(nx) > 0.5) uv.setXY(k, (nx > 0 ? y : sy - y) / 6, (z - lobby) / 6);
    else uv.setXY(k, (ny > 0 ? sx - x : x) / 6, (z - lobby) / 6);
  }
}

// which way each rotor turns, the same order as spin in core/quad.h: +1 counterclockwise seen from above
const SPIN = [1, 1, -1, -1];
// the rotors spin at up to 838 rad/s, far past what a 60 fps screen can show (it would look frozen or backwards),
// so the blades are drawn turning at a small share of that and fade into a blur disc as the motor speeds up
const BLADE_SHOWN = 0.04, MAX_ROTOR = 838;

// the quad: body, battery, gps mast, landing legs, an arm out to each rotor with a motor and a two blade prop on it,
// all at the physics' rotor positions
export function buildDrone(sim: Drone) {
  const drone = new THREE.Group();

  // the surfaces: glossy plastic shell, matte carbon arms, brushed metal motors
  const shell = new THREE.MeshStandardMaterial({ color: 0x24282e, roughness: 0.35, metalness: 0.1 });
  const carbon = new THREE.MeshStandardMaterial({ color: 0x15171a, roughness: 0.55 });
  const metal = new THREE.MeshStandardMaterial({ color: 0xb8bcc2, roughness: 0.3, metalness: 1 });
  // front props orange and back ones gray, so you can tell which way it faces
  const frontProp = new THREE.MeshStandardMaterial({ color: 0xff8a2a, roughness: 0.45 });
  const backProp = new THREE.MeshStandardMaterial({ color: 0x9aa3ad, roughness: 0.45 });

  // the body: a rounded shell on a flat deck, with the battery strapped under it
  const top = new THREE.Mesh(new THREE.SphereGeometry(1, 32, 16, 0, Math.PI * 2, 0, Math.PI / 2), shell);
  // the half sphere is built round three's y, so a quarter turn stands the dome up
  top.rotation.x = Math.PI / 2;
  top.scale.set(0.11, 0.04, 0.07);
  const deck = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, 0.012, 32), carbon);
  deck.scale.set(0.11, 1, 0.07);
  deck.rotation.x = Math.PI / 2;
  const battery = new THREE.Mesh(
    new THREE.BoxGeometry(0.13, 0.05, 0.03),
    new THREE.MeshStandardMaterial({ color: 0x2f5fa8, roughness: 0.5 }),
  );
  battery.position.z = -0.022;
  drone.add(top, deck, battery);

  // the gps puck on a short mast at the back, where real ones sit away from the motors' magnetic noise
  const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.003, 0.06), carbon);
  mast.rotation.x = Math.PI / 2;
  mast.position.set(-0.06, 0, 0.06);
  const puck = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.01, 24), shell);
  puck.rotation.x = Math.PI / 2;
  puck.position.set(-0.06, 0, 0.092);
  drone.add(mast, puck);

  // two landing legs each side, splayed out and down, with a skid along the bottom
  for (const side of [-1, 1]) {
    for (const end of [-1, 1]) {
      const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.004, 0.11), carbon);
      leg.position.set(end * 0.05, side * 0.07, -0.06);
      // cylinders stand along their own y, so a quarter turn stands it up, then it splays outward
      leg.rotation.x = Math.PI / 2 + side * 0.45;
      drone.add(leg);
    }
    const skid = new THREE.Mesh(new THREE.CylinderGeometry(0.005, 0.005, 0.16), carbon);
    skid.rotation.z = Math.PI / 2;
    skid.position.set(0, side * 0.095, -0.11);
    drone.add(skid);
  }

  const props: THREE.Group[] = [];
  const blurs: THREE.MeshBasicMaterial[] = [];
  for (let i = 0; i < 4; i++) {
    const x = sim._rotor_pos(i, 0), y = sim._rotor_pos(i, 1), z = sim._rotor_pos(i, 2);
    const front = x > 0;

    // round carbon arm from the center out to the motor, turned to point at it
    const len = Math.hypot(x, y);
    const arm = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, len, 12), carbon);
    arm.position.set(x / 2, y / 2, z - 0.012);
    arm.rotation.z = Math.atan2(y, x) - Math.PI / 2;
    drone.add(arm);

    // the motor: a metal bell on the arm's end, and a light under it, white at the front and red at the back
    const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.02, 0.026, 20), metal);
    motor.rotation.x = Math.PI / 2;
    motor.position.set(x, y, z - 0.003);
    const led = new THREE.Mesh(
      new THREE.SphereGeometry(0.006, 8, 6),
      new THREE.MeshBasicMaterial({ color: front ? 0xffffff : 0xff2a2a }),
    );
    led.position.set(x, y, z - 0.02);
    drone.add(motor, led);

    // the prop: a hub and two blades with a little twist, which is what pushes the air down
    const prop = new THREE.Group();
    prop.position.set(x, y, z + 0.013);
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.008, 12), metal);
    hub.rotation.x = Math.PI / 2;
    prop.add(hub);
    for (const dir of [-1, 1]) {
      const blade = new THREE.Mesh(new THREE.BoxGeometry(0.095, 0.016, 0.002), front ? frontProp : backProp);
      blade.position.x = dir * 0.05;
      blade.rotation.x = dir * SPIN[i] * 0.2;
      prop.add(blade);
    }
    drone.add(prop);
    props.push(prop);

    // the blur disc a spinning prop looks like, more solid the faster it turns. it doesn't cast a shadow
    const blur = new THREE.MeshBasicMaterial({
      color: front ? 0xff8a2a : 0x9aa3ad, transparent: true, opacity: 0, depthWrite: false, side: THREE.DoubleSide,
    });
    const disc = new THREE.Mesh(new THREE.CircleGeometry(0.1, 32), blur);
    disc.position.set(x, y, z + 0.013);
    drone.add(disc);
    blurs.push(blur);
  }

  // every solid part throws a shadow on the ground and buildings
  drone.traverse((o) => {
    if (o instanceof THREE.Mesh && !(o.material as THREE.Material).transparent) o.castShadow = true;
  });
  // kept on the drone so spinProps can find them each frame
  drone.userData = { props, blurs };
  return drone;
}

// turns the props by this frame's rotor speeds (rad/s) and sets how solid each blur disc looks
export function spinProps(drone: THREE.Group, rotors: ArrayLike<number>, frameS: number) {
  const { props, blurs } = drone.userData as { props: THREE.Group[]; blurs: THREE.MeshBasicMaterial[] };
  for (let i = 0; i < 4; i++) {
    props[i].rotation.z += SPIN[i] * rotors[i] * BLADE_SHOWN * frameS;
    blurs[i].opacity = 0.35 * Math.min(1, rotors[i] / MAX_ROTOR);
  }
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
  const mat = new THREE.MeshBasicMaterial({ color: COLORS.estimate, transparent: true, opacity: 0.28, depthWrite: false });
  // all see-through blue, and a guess shouldn't throw a shadow. its blur discs are hidden so they don't cover the real props
  ghost.traverse((o) => {
    if (o instanceof THREE.Mesh) {
      if ((o.material as THREE.Material).transparent) o.visible = false;
      o.material = mat;
      o.castShadow = false;
    }
  });
  // a bit bigger so it shows around the real drone when the two sit on top of each other
  ghost.scale.setScalar(1.15);
  // its props never spin, so it doesn't need the prop list buildDrone keeps for spinProps
  ghost.userData = {};
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
  const box = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: 0x8796a8, roughness: 0.6, transparent: true, opacity: 0.35 }));
  // it still throws a shadow, it's a real box even if the drone doesn't know it yet
  box.castShadow = true;
  box.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo), new THREE.LineBasicMaterial({ color: 0x1d2733 })));
  box.position.set((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2);
  return box;
}
