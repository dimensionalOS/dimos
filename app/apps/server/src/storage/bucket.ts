import { S3Client } from "bun";
import { env } from "../env";

// Bun's native S3 client — no @aws-sdk dependency. Points at MinIO locally and
// the Railway bucket in production (same S3 API for both).
const bucket = new S3Client({
  endpoint: env.S3_ENDPOINT,
  region: env.S3_REGION,
  accessKeyId: env.S3_ACCESS_KEY_ID,
  secretAccessKey: env.S3_SECRET_ACCESS_KEY,
  bucket: env.S3_BUCKET,
});

export async function writeImage(
  key: string,
  bytes: ArrayBuffer,
  contentType: string,
): Promise<void> {
  await bucket.write(key, bytes, { type: contentType });
}

// Generic object write (no image-type assumption) — used for splat files
// (.ply/.spz/.splat/.ksplat/.sog), which are binary blobs.
export async function writeObject(
  key: string,
  bytes: ArrayBuffer,
  contentType: string,
): Promise<void> {
  await bucket.write(key, bytes, { type: contentType });
}

// Time-limited GET URL the browser can hit directly. The raw key never leaves
// the server. Bun.presign is synchronous; we wrap it in a promise to match the
// ApiContext.presignGet contract. `expiresIn` defaults to 1h (images); splats
// pass a longer TTL since they're large and slow to stream.
export function presignGet(key: string, expiresIn = 3600): Promise<string> {
  return Promise.resolve(bucket.presign(key, { method: "GET", expiresIn }));
}
