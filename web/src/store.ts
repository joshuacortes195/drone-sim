// what the panel shows that only the sim knows, written by main.ts and read by react
export interface Readout {
  windX: number; // the wind the drone feels right now, m/s east
  windY: number; // and m/s north
  windZ: number; // and m/s up
  crashed: boolean;
  demo: boolean; // true while it flies the built in tour, false once someone picks a goal
  note: string; // a short message after a tap that didn't work, empty when there's nothing to say
}

let readout: Readout = { windX: 0, windY: 0, windZ: 0, crashed: false, demo: true, note: '' };
const listeners = new Set<() => void>();

// swaps in the changed numbers and tells every subscribed component
export function setReadout(change: Partial<Readout>) {
  readout = { ...readout, ...change };
  for (const l of listeners) l();
}

// hooks for useSyncExternalStore
export function subscribe(l: () => void) {
  listeners.add(l);
  return () => listeners.delete(l);
}
export const getReadout = () => readout;
