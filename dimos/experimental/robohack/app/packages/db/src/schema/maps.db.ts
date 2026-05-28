import {
  doublePrecision,
  integer,
  pgTable,
  text,
  timestamp,
} from "drizzle-orm/pg-core";

// 2D occupancy-map snapshots from the robot (the dimos global_costmap rendered
// to a PNG). `imageKey` is the S3 object; resolution + origin + width/height let
// the web map world coordinates onto image pixels. We only ever read the newest.
export const maps = pgTable("maps", {
  id: text("id").primaryKey(),
  imageKey: text("image_key").notNull(),
  resolution: doublePrecision("resolution").notNull(),
  originX: doublePrecision("origin_x").notNull(),
  originY: doublePrecision("origin_y").notNull(),
  width: integer("width").notNull(),
  height: integer("height").notNull(),
  createdAt: timestamp("created_at").notNull().defaultNow(),
});
