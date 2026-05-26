import { z } from "zod";

// Startup-time env validation. Required keys fail the parse immediately so the
// server refuses to boot into a half-configured state.
const envSchema = z.object({
  NODE_ENV: z.enum(["development", "test", "production"]).default("development"),
  PORT: z.coerce.number().int().positive().default(4471),

  // Postgres — auth tables + messages.
  DATABASE_URL: z.string().url(),

  // Better Auth secret (HMAC). 32+ random bytes.
  BETTER_AUTH_SECRET: z.string().min(1),

  // Public origin the browser sees (the Caddy gateway). Better Auth baseURL +
  // trustedOrigin.
  APP_URL: z.string().url().default("http://localhost:4470"),

  // S3-compatible object storage (MinIO locally, Railway bucket in prod).
  S3_ENDPOINT: z.string().url(),
  S3_REGION: z.string().default("auto"),
  S3_ACCESS_KEY_ID: z.string().min(1),
  S3_SECRET_ACCESS_KEY: z.string().min(1),
  S3_BUCKET: z.string().min(1),

  // Shared secret the robot presents (Authorization: Bearer) to POST frames to
  // /api/robot/frame. Optional so the server still boots if unset — the ingest
  // endpoint just rejects every request until it's configured.
  ROBOT_INGEST_TOKEN: z.string().optional(),
});

export const env = envSchema.parse(Bun.env);
export type Env = typeof env;
