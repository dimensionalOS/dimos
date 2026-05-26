import { z } from "zod";

// Short, prefixed, sortable-ish id. Avoids an extra dependency while keeping
// human-readable ids (e.g. "msg_lqf3k2a9z1"). Good enough for the sample.
export function newId(prefix: string): string {
  const rand = crypto.randomUUID().replace(/-/g, "").slice(0, 16);
  return `${prefix}_${rand}`;
}

// A message as the API returns it to the web client. `imageUrl` is a freshly
// presigned GET URL (or null) — the raw S3 key never leaves the server.
export const messageSchema = z.object({
  id: z.string(),
  body: z.string(),
  imageUrl: z.string().nullable(),
  authorName: z.string(),
  createdAt: z.string(),
});
export type Message = z.infer<typeof messageSchema>;

// Input for creating a message. `imageKey` is the key returned by the
// /api/upload/image endpoint after the file lands in the bucket.
export const createMessageInput = z.object({
  body: z.string().min(1).max(500),
  imageKey: z.string().nullable().default(null),
});
export type CreateMessageInput = z.infer<typeof createMessageInput>;
