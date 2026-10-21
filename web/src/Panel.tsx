import { useState, useSyncExternalStore } from 'react';
import { getReadout, subscribe } from './store';

// what a tap on the 3d view does
export type TapMode = 'goal' | 'box';

// what the panel can ask the sim to do, handed in by main.ts
export interface Actions {
  setWind(breeze: number, gusts: number): void;
  setNoisyGps(on: boolean): void;
  setTruth(on: boolean): void;
  setTapMode(mode: TapMode): void;
  reset(): void;
}

// slider ranges: breeze up to 8 m/s (the hover test's breeze), gusts up to 3 m/s
const BREEZE_MAX = 8, GUSTS_MAX = 3;

// phones start with the panel folded up so the sim gets the whole screen
const TOUCH = window.matchMedia('(pointer: coarse)').matches;

// the wind right now: speed plus an arrow pointing the way it blows, north up like the map view
function WindReadout() {
  const r = useSyncExternalStore(subscribe, getReadout);
  const speed = Math.hypot(r.windX, r.windY, r.windZ);
  // svg y points down, so north (+y) needs the angle flipped
  const deg = (-Math.atan2(r.windY, r.windX) * 180) / Math.PI;
  return (
    <div className="wind">
      <svg width="22" height="22" viewBox="-11 -11 22 22" aria-hidden="true">
        <circle r="10" className="ring" />
        {speed > 0.05 ? <path d="M7 0 L-2 -5 L0 0 L-2 5 Z" transform={`rotate(${deg})`} /> : null}
      </svg>
      <span>wind {speed.toFixed(1)} m/s</span>
    </div>
  );
}

// what the drone is up to, or why the last tap didn't do anything
function Status() {
  const r = useSyncExternalStore(subscribe, getReadout);
  const text = r.note || (r.demo ? 'flying the tour' : 'flying to your goal, reset to restart the tour');
  return <p className="status" aria-live="polite">{text}</p>;
}

// big banner across the top once the drone hits something, only re-renders when that flips
export function CrashBanner({ onReset }: { onReset: () => void }) {
  const crashed = useSyncExternalStore(subscribe, () => getReadout().crashed);
  return crashed ? (
    <div className="crash" role="alert">
      crashed <button onClick={onReset}>reset</button>
    </div>
  ) : null;
}

// the control panel in the top right corner: wind sliders, sensor toggles, what a tap does, reset
export function Panel({ actions }: { actions: Actions }) {
  const [open, setOpen] = useState(!TOUCH);
  const [breeze, setBreeze] = useState(0);
  const [gusts, setGusts] = useState(0);
  const [noisyGps, setNoisyGps] = useState(false);
  const [truth, setTruth] = useState(false);
  const [tapMode, setTapMode] = useState<TapMode>('goal');

  // moves the breeze slider and tells the sim
  function changeBreeze(v: number) {
    setBreeze(v);
    actions.setWind(v, gusts);
  }

  // moves the gusts slider and tells the sim
  function changeGusts(v: number) {
    setGusts(v);
    actions.setWind(breeze, v);
  }

  // flips the everyday gps on or off
  function toggleGps() {
    setNoisyGps(!noisyGps);
    actions.setNoisyGps(!noisyGps);
  }

  // flips between flying on the estimate and flying on the real state
  function toggleTruth() {
    setTruth(!truth);
    actions.setTruth(!truth);
  }

  // picks what the next tap does
  function pickTap(mode: TapMode) {
    setTapMode(mode);
    actions.setTapMode(mode);
  }

  return (
    <>
      <CrashBanner onReset={actions.reset} />
      <section className="controls">
        <header>
          <h1>controls</h1>
          <button onClick={() => setOpen(!open)} aria-expanded={open}>
            {open ? 'hide' : 'show'}
          </button>
        </header>
        {open ? (
          <div className="body">
            <fieldset className="tap">
              <legend>tap the city to</legend>
              <button aria-pressed={tapMode === 'goal'} onClick={() => pickTap('goal')}>
                set a goal
              </button>
              <button aria-pressed={tapMode === 'box'} onClick={() => pickTap('box')}>
                drop a box
              </button>
            </fieldset>
            <label>
              breeze from the west {breeze.toFixed(1)} m/s
              <input type="range" min={0} max={BREEZE_MAX} step={0.5} value={breeze} onChange={(e) => changeBreeze(+e.target.value)} />
            </label>
            <label>
              gusts {gusts.toFixed(2)} m/s
              <input type="range" min={0} max={GUSTS_MAX} step={0.25} value={gusts} onChange={(e) => changeGusts(+e.target.value)} />
            </label>
            <WindReadout />
            <label className="check">
              <input type="checkbox" checked={noisyGps} onChange={toggleGps} />
              everyday gps (meters off) instead of rtk
            </label>
            <label className="check">
              <input type="checkbox" checked={truth} onChange={toggleTruth} />
              fly on the real position (truth)
            </label>
            <button className="reset" onClick={actions.reset}>
              reset
            </button>
          </div>
        ) : null}
        <Status />
      </section>
    </>
  );
}
