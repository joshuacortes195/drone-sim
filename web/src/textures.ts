import * as THREE from 'three';

// all the surface pictures are painted on canvases at load, so the page ships no image files

// a canvas of the given size and its 2d pen
function canvas(w: number, h: number) {
  const c = document.createElement('canvas');
  c.width = w;
  c.height = h;
  return [c, c.getContext('2d')!] as const;
}

// a small repeatable random generator, so the painted city looks the same on every load
function rng(seed: number) {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

// sprinkles light and dark speckle over everything already painted, so flat colors read as asphalt or concrete
function grain(ctx: CanvasRenderingContext2D, w: number, h: number, amount: number, seed: number) {
  const img = ctx.getImageData(0, 0, w, h);
  const r = rng(seed);
  for (let i = 0; i < img.data.length; i += 4) {
    const n = (r() - 0.5) * amount;
    img.data[i] += n;
    img.data[i + 1] += n;
    img.data[i + 2] += n;
  }
  ctx.putImageData(img, 0, 0);
}

// turns a painted canvas into a texture, color ones get marked srgb so the lighting math gets real colors
function toTexture(c: HTMLCanvasElement, color: boolean) {
  const t = new THREE.CanvasTexture(c);
  if (color) t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = 8;
  return t;
}

// city layout from core/map.cpp: lots 8 m wide on a 12 m grid, 4 m streets between them
const LOT = 12, HALF_LOT = 4;
// meters of ground the city picture covers, centered on home
export const CITY_M = 72;

// the city floor: streets with lane marks and crosswalks, sidewalks round every lot, a pad at home and grass where a lot
// has no building. boxes is the building list (6 numbers each) so it knows which lots are empty
export function cityGround(boxes: Float64Array) {
  const PX = 2048;
  const s = PX / CITY_M;
  const [c, ctx] = canvas(PX, PX);
  // world meters to canvas pixels, north up
  const X = (x: number) => (x + CITY_M / 2) * s;
  const Y = (y: number) => (CITY_M / 2 - y) * s;
  const rect = (x0: number, y0: number, x1: number, y1: number) => ctx.fillRect(X(x0), Y(y1), (x1 - x0) * s, (y1 - y0) * s);

  // grass round the outside, asphalt over the whole city out to the far side of the outer streets
  ctx.fillStyle = '#6f7d4f';
  ctx.fillRect(0, 0, PX, PX);
  ctx.fillStyle = '#3d3f42';
  rect(-32, -32, 32, 32);

  // street center lines, dashed, skipping the crossings
  const streets = [-30, -18, -6, 6, 18, 30];
  ctx.fillStyle = '#d9c46a';
  for (const c0 of streets)
    for (let a = -32; a < 32; a += 3) {
      if (streets.some((c1) => Math.abs(a + 0.75 - c1) < 2.6)) continue;
      rect(a, c0 - 0.06, a + 1.5, c0 + 0.06);
      rect(c0 - 0.06, a, c0 + 0.06, a + 1.5);
    }

  // zebra crosswalks on each side of every crossing
  ctx.fillStyle = '#d8d8d2';
  for (const cx of streets)
    for (const cy of streets)
      for (const side of [-1, 1])
        for (let k = -1.6; k <= 1.4; k += 0.6) {
          // the outer crossings have no street past them, so no crosswalk out on the grass
          if (Math.abs(cy + side * 2.3) < 31) rect(cx + k, cy + side * 2.3 - 0.4, cx + k + 0.3, cy + side * 2.3 + 0.4);
          if (Math.abs(cx + side * 2.3) < 31) rect(cx + side * 2.3 - 0.4, cy + k, cx + side * 2.3 + 0.4, cy + k + 0.3);
        }

  // which lots have a building, by where each building's middle falls
  const built = new Set<string>();
  for (let b = 0; b < boxes.length; b += 6) {
    const i = Math.round((boxes[b] + boxes[b + 3]) / 2 / LOT), j = Math.round((boxes[b + 1] + boxes[b + 4]) / 2 / LOT);
    built.add(`${i},${j}`);
  }

  // every lot: a sidewalk edge with a curb, then a plaza, a little park or the home pad inside it
  for (let i = -2; i <= 2; i++)
    for (let j = -2; j <= 2; j++) {
      const cx = LOT * i, cy = LOT * j;
      ctx.fillStyle = '#8e8d88';
      rect(cx - HALF_LOT - 0.15, cy - HALF_LOT - 0.15, cx + HALF_LOT + 0.15, cy + HALF_LOT + 0.15);
      ctx.fillStyle = '#b4b1a8';
      rect(cx - HALF_LOT, cy - HALF_LOT, cx + HALF_LOT, cy + HALF_LOT);
      if (i === 0 && j === 0) {
        homePad(ctx, X(cx), Y(cy), s);
      } else if (!built.has(`${i},${j}`)) {
        // grass with a path across it
        ctx.fillStyle = '#5f7a3c';
        rect(cx - 3, cy - 3, cx + 3, cy + 3);
        ctx.fillStyle = '#a99f86';
        rect(cx - 3, cy - 0.4, cx + 3, cy + 0.4);
      } else {
        // plaza paving: a grid of slab joints
        ctx.strokeStyle = 'rgba(70, 68, 62, 0.35)';
        ctx.lineWidth = 1;
        for (let k = -HALF_LOT; k <= HALF_LOT; k += 1) {
          ctx.beginPath();
          ctx.moveTo(X(cx + k), Y(cy - HALF_LOT));
          ctx.lineTo(X(cx + k), Y(cy + HALF_LOT));
          ctx.moveTo(X(cx - HALF_LOT), Y(cy + k));
          ctx.lineTo(X(cx + HALF_LOT), Y(cy + k));
          ctx.stroke();
        }
      }
    }

  grain(ctx, PX, PX, 22, 7);
  return toTexture(c, true);
}

// the takeoff pad at home: dark square, white ring and an h
function homePad(ctx: CanvasRenderingContext2D, cx: number, cy: number, s: number) {
  ctx.fillStyle = '#2e3236';
  ctx.fillRect(cx - 3.2 * s, cy - 3.2 * s, 6.4 * s, 6.4 * s);
  ctx.strokeStyle = '#e8e6df';
  ctx.lineWidth = 0.18 * s;
  ctx.beginPath();
  ctx.arc(cx, cy, 2.6 * s, 0, Math.PI * 2);
  ctx.stroke();
  ctx.fillStyle = '#e8e6df';
  ctx.fillRect(cx - 1.1 * s, cy - 1.4 * s, 0.4 * s, 2.8 * s);
  ctx.fillRect(cx + 0.7 * s, cy - 1.4 * s, 0.4 * s, 2.8 * s);
  ctx.fillRect(cx - 0.7 * s, cy - 0.2 * s, 1.4 * s, 0.4 * s);
}

// plain grass for the land outside the city, tiled
export function grass() {
  const [c, ctx] = canvas(256, 256);
  ctx.fillStyle = '#6f7d4f';
  ctx.fillRect(0, 0, 256, 256);
  grain(ctx, 256, 256, 30, 3);
  return toTexture(c, true);
}

// gravel roof, tiled every 4 m
export function roof() {
  const [c, ctx] = canvas(256, 256);
  ctx.fillStyle = '#6c6a66';
  ctx.fillRect(0, 0, 256, 256);
  grain(ctx, 256, 256, 40, 11);
  return toTexture(c, true);
}

// one building look: how the wall and the windows are painted
type Style = {
  wall: string;
  glass: string;
  // window size and spot inside one 3 m wide, 3 m tall bay, in meters
  win: [number, number, number, number];
  brick?: boolean;
  // how rough and how metal the glass is (0 to 1), walls are always rough and not metal
  glassRough: number;
  glassMetal: number;
};

export const STYLES: Style[] = [
  // glass office tower: thin mullions, nearly all glass
  { wall: '#3b4148', glass: '#41566b', win: [0.12, 0.25, 2.88, 2.95], glassRough: 0.08, glassMetal: 0.7 },
  // concrete block with punched windows
  { wall: '#a99f8e', glass: '#2f3a45', win: [0.75, 0.9, 2.25, 2.55], glassRough: 0.12, glassMetal: 0.4 },
  // brick walk up with tall windows
  { wall: '#8d5340', glass: '#2b333b', win: [0.9, 0.8, 2.1, 2.6], glassRough: 0.12, glassMetal: 0.4, brick: true },
];

// a facade picture covering 2 bays across and 2 floors up (6 m x 6 m), plus a second picture holding roughness in
// green and metalness in blue the way three.js reads them. a few windows get blinds so the repeat is harder to spot
export function facade(st: Style, seed: number) {
  const PX = 512, s = PX / 6;
  const [c, ctx] = canvas(PX, PX);
  const [m, mtx] = canvas(PX, PX);
  const r = rng(seed);

  // the wall, and walls are rough and not metal
  ctx.fillStyle = st.wall;
  ctx.fillRect(0, 0, PX, PX);
  mtx.fillStyle = 'rgb(0, 230, 0)';
  mtx.fillRect(0, 0, PX, PX);

  // brick courses: a darker mortar line every 7.5 cm with offset joints
  if (st.brick) {
    ctx.fillStyle = 'rgba(60, 40, 32, 0.45)';
    for (let row = 0, y = 0; y < PX; row++, y += 0.075 * s) {
      ctx.fillRect(0, y, PX, 1.2);
      for (let x = (row % 2) * 0.11 * s; x < PX; x += 0.22 * s) ctx.fillRect(x, y, 1.2, 0.075 * s);
    }
  }

  for (let bx = 0; bx < 2; bx++)
    for (let fy = 0; fy < 2; fy++) {
      // canvas y runs down, the floor slab line sits along the bottom of each floor
      const x0 = bx * 3 * s, top = (1 - fy) * 3 * s;
      ctx.fillStyle = 'rgba(0, 0, 0, 0.18)';
      ctx.fillRect(0, top + 3 * s - 0.12 * s, PX, 0.12 * s);

      // the window: glass with a lighter frame round it, some with half drawn blinds
      const [wx0, wy0, wx1, wy1] = st.win;
      const px = x0 + wx0 * s, py = top + (3 - wy1) * s, pw = (wx1 - wx0) * s, ph = (wy1 - wy0) * s;
      ctx.fillStyle = 'rgba(230, 230, 225, 0.8)';
      ctx.fillRect(px - 3, py - 3, pw + 6, ph + 6);
      ctx.fillStyle = st.glass;
      ctx.fillRect(px, py, pw, ph);
      // a soft sky glint toward the top of each pane
      const g = ctx.createLinearGradient(0, py, 0, py + ph);
      g.addColorStop(0, 'rgba(200, 220, 240, 0.25)');
      g.addColorStop(1, 'rgba(200, 220, 240, 0)');
      ctx.fillStyle = g;
      ctx.fillRect(px, py, pw, ph);
      if (r() < 0.3) {
        ctx.fillStyle = 'rgba(210, 200, 180, 0.85)';
        ctx.fillRect(px, py, pw, ph * (0.3 + 0.5 * r()));
      }
      // a mullion down the middle of the glass
      ctx.fillStyle = 'rgba(230, 230, 225, 0.8)';
      ctx.fillRect(px + pw / 2 - 2, py, 4, ph);
      // the glass is smooth and a bit metal so it picks up the sky
      mtx.fillStyle = `rgb(0, ${Math.round(st.glassRough * 255)}, ${Math.round(st.glassMetal * 255)})`;
      mtx.fillRect(px, py, pw, ph);
    }

  grain(ctx, PX, PX, 14, seed);
  return { map: toTexture(c, true), rm: toTexture(m, false) };
}
