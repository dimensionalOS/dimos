import { Costmap, EncodedSomething, Grid, Path, Vector } from "./types.ts"

export function decode(data: EncodedSomething) {
    console.log("decoding", data)
    if (data.type == "costmap") {
        return Costmap.decode(data)
    }
    if (data.type == "vector") {
        return Vector.decode(data)
    }
    if (data.type == "grid") {
        return Grid.decode(data)
    }
    if (data.type == "path") {
        return Path.decode(data)
    }

    return "UNKNOWN"
}
