import { pgTable, text, timestamp } from "drizzle-orm/pg-core";
import { user } from "./auth.db";

// The one product table for the sample. `imageKey` is the S3 object key (the
// presigned URL is derived at read time); null when the message has no image.
export const messages = pgTable("messages", {
  id: text("id").primaryKey(),
  body: text("body").notNull(),
  imageKey: text("image_key"),
  authorId: text("author_id")
    .notNull()
    .references(() => user.id, { onDelete: "cascade" }),
  createdAt: timestamp("created_at").notNull().defaultNow(),
});
