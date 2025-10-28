import { EncodedOptimizedGrid, OptimizedGrid } from './optimizedCostmap';

export type EncodedVector = Encoded<"vector"> & {
  c: number[];
};

export class Vector {
  coords: number[];
  constructor(...coords: number[]) {
    this.coords = coords;
  }

  static decode(data: EncodedVector): Vector {
    return new Vector(...data.c);
  }
}

export interface LatLon {
  lat: number;
  lon: number;
  alt?: number;
}

export type EncodedPath = Encoded<"path"> & {
  points: Array<[number, number]>;
};

export class Path {
  constructor(public coords: Array<[number, number]>) {}

  static decode(data: EncodedPath): Path {
    return new Path(data.points);
  }
}

export type EncodedCostmap = Encoded<"costmap"> & {
  grid: EncodedOptimizedGrid;
  origin: EncodedVector;
  resolution: number;
  origin_theta: number;
};

export class Costmap {
  constructor(
    public grid: Grid,
    public origin: Vector,
    public resolution: number,
    public origin_theta: number,
  ) {
    this.grid = grid;
    this.origin = origin;
    this.resolution = resolution;
    this.origin_theta = origin_theta;
  }

  private static decoder: OptimizedGrid | null = null;

  static decode(data: EncodedCostmap): Costmap {
    // Use a singleton decoder to maintain state for delta updates
    if (!Costmap.decoder) {
      Costmap.decoder = new OptimizedGrid();
    }

    const float32Data = Costmap.decoder.decode(data.grid);
    const shape = data.grid.shape;

    // Create a Grid object from the decoded data
    const grid = new Grid(float32Data, shape);

    return new Costmap(
      grid,
      Vector.decode(data.origin),
      data.resolution,
      data.origin_theta,
    );
  }
}

export class Grid {
  constructor(
    public data: Float32Array | Float64Array | Int32Array | Int8Array,
    public shape: number[],
  ) {}
}

export type Drawable = Costmap | Vector | Path;

export type Encoded<T extends string> = {
  type: T;
};

export interface FullStateData {
  costmap?: EncodedCostmap;
  robot_pose?: EncodedVector;
  gps_location?: LatLon;
  gps_travel_goal_points?: LatLon[];
  path?: EncodedPath;
}

export interface TwistCommand {
  linear: {
    x: number;
    y: number;
    z: number;
  };
  angular: {
    x: number;
    y: number;
    z: number;
  };
}

export interface AppState {
  costmap: Costmap | null;
  robotPose: Vector | null;
  gpsLocation: LatLon | null;
  gpsTravelGoalPoints: LatLon[] | null;
  path: Path | null;
}

export type AppAction =
  | { type: "SET_COSTMAP"; payload: Costmap }
  | { type: "SET_ROBOT_POSE"; payload: Vector }
  | { type: "SET_GPS_LOCATION"; payload: LatLon }
  | { type: "SET_GPS_TRAVEL_GOAL_POINTS"; payload: LatLon[] }
  | { type: "SET_PATH"; payload: Path }
  | { type: "SET_FULL_STATE"; payload: Partial<AppState> };
