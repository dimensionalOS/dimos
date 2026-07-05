/* tslint:disable */
/* eslint-disable */

export class DemoPlanner {
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Costmap raster (i8 costs, row-major) + metadata for rendering.
     */
    costmap_cells(): Int8Array;
    costmap_height(): number;
    costmap_origin_x(): number;
    costmap_origin_y(): number;
    costmap_resolution(): number;
    costmap_width(): number;
    constructor(resolution: number, half_extent: number);
    /**
     * Plan toward the goal; returns flat [x0,y0,yaw0, x1,y1,yaw1, ...].
     */
    plan(robot_x: number, robot_y: number, robot_yaw: number, goal_x: number, goal_y: number, speed: number): Float32Array;
    /**
     * Rebuild the costmap from flat [x0,y0,z0, x1,y1,z1, ...] points around
     * the robot.
     */
    update_terrain(points: Float32Array, robot_x: number, robot_y: number, robot_z: number): void;
}

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
    readonly memory: WebAssembly.Memory;
    readonly __wbg_demoplanner_free: (a: number, b: number) => void;
    readonly demoplanner_costmap_cells: (a: number) => [number, number];
    readonly demoplanner_costmap_height: (a: number) => number;
    readonly demoplanner_costmap_origin_x: (a: number) => number;
    readonly demoplanner_costmap_origin_y: (a: number) => number;
    readonly demoplanner_costmap_resolution: (a: number) => number;
    readonly demoplanner_costmap_width: (a: number) => number;
    readonly demoplanner_new: (a: number, b: number) => number;
    readonly demoplanner_plan: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => [number, number];
    readonly demoplanner_update_terrain: (a: number, b: number, c: number, d: number, e: number, f: number) => void;
    readonly __wbindgen_externrefs: WebAssembly.Table;
    readonly __wbindgen_free: (a: number, b: number, c: number) => void;
    readonly __wbindgen_malloc: (a: number, b: number) => number;
    readonly __wbindgen_start: () => void;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;

/**
 * Instantiates the given `module`, which can either be bytes or
 * a precompiled `WebAssembly.Module`.
 *
 * @param {{ module: SyncInitInput }} module - Passing `SyncInitInput` directly is deprecated.
 *
 * @returns {InitOutput}
 */
export function initSync(module: { module: SyncInitInput } | SyncInitInput): InitOutput;

/**
 * If `module_or_path` is {RequestInfo} or {URL}, makes a request and
 * for everything else, calls `WebAssembly.instantiate` directly.
 *
 * @param {{ module_or_path: InitInput | Promise<InitInput> }} module_or_path - Passing `InitInput` directly is deprecated.
 *
 * @returns {Promise<InitOutput>}
 */
export default function __wbg_init (module_or_path?: { module_or_path: InitInput | Promise<InitInput> } | InitInput | Promise<InitInput>): Promise<InitOutput>;
