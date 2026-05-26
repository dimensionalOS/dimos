import { pgTable, text, timestamp } from "drizzle-orm/pg-core";

// Robot/offline-produced 3D Gaussian splats. The reconstruction box POSTs a
// .ply/.spz/.splat/.ksplat/.sog to /api/robot/splat (token auth); `splatKey` is
// the S3 object key, `name` is an optional human label, `format` is the file
// extension (so the viewer knows what it's loading).
export const splats = pgTable("splats", {
  id: text("id").primaryKey(),
  splatKey: text("splat_key").notNull(),
  name: text("name"),
  format: text("format").notNull(),
  createdAt: timestamp("created_at").notNull().defaultNow(),
});
