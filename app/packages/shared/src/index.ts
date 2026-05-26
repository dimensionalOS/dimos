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

// A robot-captured frame as returned to the web client. `imageUrl` is a freshly
// presigned GET URL; the raw S3 key never leaves the server. `poseX`/`poseY`
// are the robot's world position when captured (for placing a pin on the map),
// `label` is what was detected/captured (e.g. "plant").
export const frameSchema = z.object({
  id: z.string(),
  imageUrl: z.string(),
  note: z.string().nullable(),
  label: z.string().nullable(),
  poseX: z.number().nullable(),
  poseY: z.number().nullable(),
  createdAt: z.string(),
});
export type Frame = z.infer<typeof frameSchema>;

// A 3D Gaussian splat as returned to the web client. `splatUrl` is a freshly
// presigned GET URL (long TTL — the files are big and slow to stream); the raw
// S3 key never leaves the server. `format` is the file extension so the viewer
// knows what it's loading (spz, ply, splat, ksplat, sog).
export const splatSchema = z.object({
  id: z.string(),
  splatUrl: z.string(),
  name: z.string().nullable(),
  format: z.string(),
  createdAt: z.string(),
});
export type Splat = z.infer<typeof splatSchema>;

// The latest 2D occupancy map snapshot (the dimos global_costmap, rendered to a
// PNG by the robot). `resolution` (m/cell) + `origin` let the web place world
// coordinates onto the image: col = (x - originX) / resolution, row similarly.
export const mapSnapshotSchema = z.object({
  imageUrl: z.string(),
  resolution: z.number(),
  originX: z.number(),
  originY: z.number(),
  width: z.number(),
  height: z.number(),
  createdAt: z.string(),
});
export type MapSnapshot = z.infer<typeof mapSnapshotSchema>;
