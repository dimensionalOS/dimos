import { pgTable, text, timestamp } from "drizzle-orm/pg-core";

// Robot-captured frames. The robot POSTs a JPEG to /api/robot/frame (token
// auth); `imageKey` is the S3 object key, `note` is an optional free-text tag.
export const frames = pgTable("frames", {
  id: text("id").primaryKey(),
  imageKey: text("image_key").notNull(),
  note: text("note"),
  createdAt: timestamp("created_at").notNull().defaultNow(),
});
