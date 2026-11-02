import { Sha256Schema } from "#dimos/agents/shared/protocol/media.ts";
import { Type } from "typebox";

/** An Image or PointCloud2 saved by an agent module's notify call. */
export const ExportSchema = Type.Union([
  Type.Object({ kind: Type.Literal("image"), path: Type.String(), sha256: Sha256Schema }),
  Type.Object({
    kind: Type.Literal("points"),
    path: Type.String(),
    sha256: Sha256Schema,
    /** Points written to the file. */
    points: Type.Integer(),
    /** Points in the message before decimation. */
    source_points: Type.Integer(),
  }),
]);
