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

  // Where mlxvlm is reachable for auto-trigger on frame ingest. Optional —
  // unset disables auto-analysis (manual buttons still work via the web's
  // NEXT_PUBLIC_MLXVLM_URL). E.g. https://my-mac.ngrok-free.dev
  MLXVLM_URL: z.string().url().optional(),

  // Public origin of THIS server. Required to receive analysis callbacks from
  // mlxvlm — we send it as `callback_base` in every analyze-async request so
  // mlxvlm doesn't have to be configured with our URL out of band. Without it
  // mlxvlm jobs die immediately with "missing callback_base or callback_token".
  ROBOMOO_URL: z.string().url().optional(),

  // Remote dimos agent command endpoint (behind a tunnel, e.g. ngrok). The
  // server forwards browser commands here with DIMOS_AGENT_TOKEN as a bearer.
  // Both optional — the /control feature is disabled until they're set, and the
  // URL + token never leave the server.
  DIMOS_AGENT_URL: z.string().url().optional(),
  DIMOS_AGENT_TOKEN: z.string().optional(),
});

export const env = envSchema.parse(Bun.env);
export type Env = typeof env;
