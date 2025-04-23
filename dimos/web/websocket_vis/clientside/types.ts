type EncodedVector = Encoded<"vector"> & {
    c: number[]
}

export class Vector {
    coords: number[]
    constructor(...coords: number[]) {
        this.coords = coords
    }

    static decode(data: EncodedVector): Vector {
        return new Vector(...data.c)
    }
}

type EncodedPath = Encoded<"path"> & {
    points: Array<[number, number]>
}

export class Path {
    coords: Vector[]
    constructor(coords: Vector[]) {
        this.coords = coords
    }

    static decode(data: EncodedPath): Path {
        return new Path(data.points.map((p) => new Vector(...p)))
    }
}

type EncodedCostmap = Encoded<"costmap"> & {
    grid: EncodedGrid
    origin: EncodedVector
    resolution: number
    origin_theta: number
}

export class Costmap {
    constructor(
        public grid: Grid,
        public origin: Vector,
        public resolution: number,
        public origin_theta: number,
    ) {
        this.grid = grid
        this.origin = origin
        this.resolution = resolution
        this.origin_theta = origin_theta
    }

    static decode(data: EncodedCostmap): Costmap {
        return new Costmap(
            Grid.decode(data.grid),
            Vector.decode(data.origin),
            data.resolution,
            data.origin_theta,
        )
    }
}

const DTYPE = {
    f32: Float32Array,
    f64: Float64Array,
    i32: Int32Array,
    i8: Int8Array,
}

type EncodedGrid = Encoded<"grid"> & {
    shape: [number, number]
    dtype: keyof typeof DTYPE
    compressed: boolean
    data: string
}

export class Grid {
    constructor(
        public data: Float32Array | Float64Array | Int32Array | Int8Array,
        public shape: number[],
    ) {}

    static decode(msg: EncodedGrid): Grid {
        const bytes = Uint8Array.from(atob(msg.data), (c) => c.charCodeAt(0))
        const raw = bytes
        const Arr = DTYPE[msg.dtype] || Uint8Array // fallback
        return new Grid(new Arr(raw.buffer), msg.shape)
    }
}

export type Drawable = Costmap | Vector | Path

export type Encoded<T extends string> = {
    type: T
}

export type EncodedSomething =
    | EncodedCostmap
    | EncodedVector
    | EncodedGrid
    | EncodedPath
