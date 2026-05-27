import {
  doublePrecision,
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
export const frames = pgTable("frames", {
  id: text("id").primaryKey(),
  imageKey: text("image_key").notNull(),
  note: text("note"),
  label: text("label"),
  poseX: doublePrecision("pose_x"),
  poseY: doublePrecision("pose_y"),
  embedding: jsonb("embedding").$type<number[]>(),
  createdAt: timestamp("created_at").notNull().defaultNow(),
});
