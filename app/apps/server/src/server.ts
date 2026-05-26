import { appRouter, buildContext, RPCHandler } from "@robomoo/api/server";
import { createDb, messages, user } from "@robomoo/db";
import { runMigrations } from "@robomoo/db/migrator";
import { newId } from "@robomoo/shared";
import { Hono } from "hono";
import { auth } from "./auth/auth";
import { env } from "./env";
import { handleRobotFrame } from "./http/robot";
import { handleUpload } from "./http/upload";
import { presignGet } from "./storage/bucket";

// Apply pending migrations before binding the port. Railway re-runs this on
// every deploy; already-applied migrations are no-ops.
console.log("running database migrations…");
await runMigrations(env.DATABASE_URL);
console.log("migrations applied");

const db = createDb(env.DATABASE_URL);

// Seed one demo message (authored by a login-less demo user) so the home page
// has content on a fresh database. Idempotent.
const seeded = await db.select({ id: messages.id }).from(messages).limit(1);
if (seeded.length === 0) {
  const demoUserId = "user_demo";
  await db
    .insert(user)
    .values({
      id: demoUserId,
      name: "robomoo",
      email: "demo@robomoo.local",
      emailVerified: true,
    })
    .onConflictDoNothing();
  await db
    .insert(messages)
    .values({
      id: newId("msg"),
      body: "Welcome to robomoo — sign in and post a message, with an image!",
      authorId: demoUserId,
    })
    .onConflictDoNothing();
  console.log("seeded demo message");
}

const app = new Hono();
const rpcHandler = new RPCHandler(appRouter);

app.get("/health", (c) => c.json({ ok: true }));
app.get("/", (c) => c.text("robomoo server — see /health, /rpc, /api/auth"));

// Better Auth owns every /api/auth/* path (sign-up, sign-in, session,
// sign-out). Reached same-origin through the gateway, so cookies are
// first-party.
app.on(["GET", "POST"], "/api/auth/*", (c) => auth.handler(c.req.raw));

// Session-guarded image upload → object storage.
app.post("/api/upload/image", async (c) => {
  const session = await auth.api.getSession({ headers: c.req.raw.headers });
  return handleUpload(
    c.req.raw,
    session ? { user: { id: session.user.id, name: session.user.name } } : null,
  );
});

// Token-guarded robot frame ingest → object storage + frames table.
app.post("/api/robot/frame", (c) => handleRobotFrame(c.req.raw, db));

// oRPC HTTP router. Build the context per request from the Better Auth
// session, then delegate to the oRPC fetch handler.
app.all("/rpc/*", async (c) => {
  const session = await auth.api.getSession({ headers: c.req.raw.headers });
  const context = buildContext({
    db,
    session: session
      ? { user: { id: session.user.id, name: session.user.name } }
      : null,
    presignGet,
  });
  const { matched, response } = await rpcHandler.handle(c.req.raw, {
    prefix: "/rpc",
    context,
  });
  if (matched) {
    return response;
  }
  return c.notFound();
});

const server = Bun.serve({
  port: env.PORT,
  // Dual-stack bind so Railway's private network (gateway →
  // server.railway.internal) reaches us.
  hostname: "::",
  fetch: app.fetch,
});

console.log(`server listening on :${server.port}`);

process.on("SIGTERM", () => {
  server.stop();
  process.exit(0);
});
process.on("SIGINT", () => {
  server.stop();
  process.exit(0);
});
