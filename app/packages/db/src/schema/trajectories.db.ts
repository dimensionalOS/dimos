import { pgTable, text, timestamp } from "drizzle-orm/pg-core";

// Robot odometry trajectory snapshots. The robot POSTs a JSON array of
// `{ts, x, y, theta}` points to /api/robot/trajectory (token auth); `pointsKey`
// is the S3 object holding that JSON (same key-indirection as maps). We only
// ever read the newest, and draw it as a polyline + heading marker on the map.
export const trajectories = pgTable("trajectories", {
  id: text("id").primaryKey(),
  pointsKey: text("points_key").notNull(),
  createdAt: timestamp("created_at").notNull().defaultNow(),
});
