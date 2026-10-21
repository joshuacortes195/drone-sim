// live plot of how far the estimate is from the real drone, the last 30 s at 10 samples a second (each one a 0.1 s average)
const SAMPLES = 300;
const SAMPLE_S = 0.1;

// ink and grid colors match the readout, the line is the estimate's blue
const INK = '#1d2733', MUTED = '#5b6776', GRID = 'rgba(29, 39, 51, 0.12)', LINE = '#4a6fe3';

export function buildPlot(canvas: HTMLCanvasElement) {
  const ctx = canvas.getContext('2d')!;
  // newest sample last, in cm
  const data: number[] = [];
  // where the mouse is over the plot, null when it's not
  let hoverX: number | null = null;

  // keeps the canvas sharp on high dpi screens
  function fit() {
    const r = canvas.getBoundingClientRect();
    canvas.width = Math.round(r.width * devicePixelRatio);
    canvas.height = Math.round(r.height * devicePixelRatio);
  }
  fit();
  window.addEventListener('resize', fit);

  // hover shows the error at that moment
  canvas.addEventListener('pointermove', (ev) => {
    hoverX = ev.clientX - canvas.getBoundingClientRect().left;
    draw();
  });
  canvas.addEventListener('pointerleave', () => {
    hoverX = null;
    draw();
  });

  // rounds the top of the scale up to 1, 2 or 5 times a power of ten, at least 2 cm
  function niceMax(v: number) {
    const m = Math.max(v, 2);
    const p = 10 ** Math.floor(Math.log10(m));
    for (const k of [1, 2, 5, 10]) if (m <= k * p) return k * p;
    return 10 * p;
  }

  // draws the axes, the grid, the line, the latest value and the hover readout
  function draw() {
    const dpr = devicePixelRatio;
    const w = canvas.width / dpr, h = canvas.height / dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    // plot area inside the title, axis labels and the room for the latest value on the right
    const left = 30, right = w - 44, top = 20, bottom = h - 16;
    const max = niceMax(Math.max(0, ...data));
    const x = (i: number) => right - ((data.length - 1 - i) / (SAMPLES - 1)) * (right - left);
    const y = (v: number) => bottom - (v / max) * (bottom - top);

    // title
    ctx.font = '12px ui-monospace, Menlo, monospace';
    ctx.fillStyle = INK;
    ctx.textBaseline = 'alphabetic';
    ctx.textAlign = 'left';
    ctx.fillText('gap to where it thinks it is (cm)', left, 13);

    // three faint grid lines with their values on the left
    ctx.font = '10px ui-monospace, Menlo, monospace';
    ctx.textBaseline = 'middle';
    ctx.textAlign = 'right';
    for (const f of [0, 0.5, 1]) {
      const gy = y(max * f);
      ctx.strokeStyle = GRID;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(left, gy);
      ctx.lineTo(right, gy);
      ctx.stroke();
      ctx.fillStyle = MUTED;
      ctx.fillText(String(+(max * f).toPrecision(2)), left - 4, gy);
    }

    // time labels under the plot
    ctx.textBaseline = 'alphabetic';
    ctx.textAlign = 'left';
    ctx.fillText('30 s ago', left, h - 3);
    ctx.textAlign = 'right';
    ctx.fillText('now', right, h - 3);
    if (data.length === 0) return;

    // the error line, 2 px
    ctx.strokeStyle = LINE;
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.beginPath();
    data.forEach((v, i) => (i === 0 ? ctx.moveTo(x(i), y(v)) : ctx.lineTo(x(i), y(v))));
    ctx.stroke();

    // the latest value written at the end of the line
    const last = data[data.length - 1];
    ctx.fillStyle = INK;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    ctx.font = '11px ui-monospace, Menlo, monospace';
    ctx.fillText(last.toFixed(1), right + 6, Math.min(bottom, Math.max(top, y(last))));
    canvas.setAttribute('aria-label', `where it thinks it is is ${last.toFixed(1)} cm off`);

    // hover: a line at the nearest sample with its value and how long ago it was
    if (hoverX !== null && hoverX >= left && hoverX <= right) {
      const back = Math.round(((right - hoverX) / (right - left)) * (SAMPLES - 1));
      const i = data.length - 1 - back;
      if (i >= 0) {
        ctx.strokeStyle = MUTED;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(x(i), top);
        ctx.lineTo(x(i), bottom);
        ctx.stroke();
        ctx.fillStyle = LINE;
        ctx.beginPath();
        ctx.arc(x(i), y(data[i]), 4, 0, 2 * Math.PI);
        ctx.fill();
        const label = `${data[i].toFixed(1)} cm, ${(back * SAMPLE_S).toFixed(1)} s ago`;
        ctx.font = '11px ui-monospace, Menlo, monospace';
        ctx.textBaseline = 'alphabetic';
        ctx.textAlign = x(i) > (left + right) / 2 ? 'right' : 'left';
        ctx.fillStyle = INK;
        ctx.fillText(label, x(i) + (ctx.textAlign === 'right' ? -6 : 6), top + 10);
      }
    }
  }

  // adds one sample (in meters) and redraws, the oldest drops off after 30 s
  function push(m: number) {
    data.push(m * 100);
    if (data.length > SAMPLES) data.shift();
    draw();
  }

  // empties the plot, used when the page resets
  function clear() {
    data.length = 0;
    draw();
  }

  draw();
  return { push, clear };
}
