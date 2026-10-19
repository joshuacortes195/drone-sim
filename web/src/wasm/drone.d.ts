// what the compiled drone module hands back to the page
export interface Drone {
  HEAPF64: Float64Array;
  _reset(x: number, y: number, z: number, yaw: number): void;
  _set_target(x: number, y: number, z: number, yaw: number): void;
  _advance(steps: number): void;
  _state_now(): number;
  _state_prev(): number;
  _rotor_pos(i: number, axis: number): number;
  _step_size(): number;
  _new_city(seed: number): number;
  _boxes_ptr(): number;
  _fly_to(x: number, y: number, z: number): number;
  _path_ptr(): number;
  _path_done(): number;
  _add_box(x0: number, y0: number, z0: number, x1: number, y1: number, z1: number): void;
  _path_len(): number;
  _replan_count(): number;
  _est_now(): number;
  _est_prev(): number;
  _hits_ptr(): number;
  _hits_len(): number;
  _sim_tick(): number;
}

// loads the .wasm file and resolves once the module is ready
export default function createDrone(): Promise<Drone>;
