import {
  doublePrecision,
  integer,
  jsonb,
  pgTable,
  text,
  timestamp,
} from "drizzle-orm/pg-core";

// Robot-captured frames. The robot POSTs a JPEG to /api/robot/frame (token
// auth); `imageKey` is the S3 object key, `note` is an optional free-text tag,
// `label` is the detected/captured thing, and `poseX`/`poseY` are the robot's
// world position at capture (for placing a marker on the map). `embedding` is an
// optional CLIP image vector (512-d, normalized) used for in-browser semantic
// search — null for agent-captured frames that weren't embedded.
// `run`/`position`/`angle` group a VR/3D room-scan sweep: all frames sharing a
// `run` are one scan, those sharing `run`+`position` are one 360° panorama at a
// stop, and `angle` is the heading (degrees) within that panorama.
export const frames = pgTable("frames", {
  id: text("id").primaryKey(),
  imageKey: text("image_key").notNull(),
  note: text("note"),
  label: text("label"),
  poseX: doublePrecision("pose_x"),
  poseY: doublePrecision("pose_y"),
  embedding: jsonb("embedding").$type<number[]>(),
  run: text("run"),
  position: integer("position"),
  angle: doublePrecision("angle"),
  createdAt: timestamp("created_at").notNull().defaultNow(),
});
