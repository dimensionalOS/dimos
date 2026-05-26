import { createDb, schema } from "@robomoo/db";
import { betterAuth } from "better-auth";
import { drizzleAdapter } from "better-auth/adapters/drizzle";
import { env } from "../env";

const db = createDb(env.DATABASE_URL);

// Email + password auth, Postgres-backed via the Drizzle adapter. No email
// sending in the sample, so accounts are usable immediately (autoSignIn).
export const auth = betterAuth({
  baseURL: env.APP_URL,
  secret: env.BETTER_AUTH_SECRET,
  trustedOrigins: [env.APP_URL],
  database: drizzleAdapter(db, { provider: "pg", schema }),
  emailAndPassword: {
    enabled: true,
    autoSignIn: true,
    requireEmailVerification: false,
  },
});
