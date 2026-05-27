import {
	appRouter,
	buildContext,
	groupScans,
	RPCHandler,
} from "@robomoo/api/server";
import { createDb, messages, user } from "@robomoo/db";
import { runMigrations } from "@robomoo/db/migrator";
import { newId } from "@robomoo/shared";
import { Hono } from "hono";
import { auth } from "./auth/auth";
import { env } from "./env";
import {
	handleRobotFrame,
	handleRobotFrameAnalysis,
	handleRobotMap,
	handleRobotSplat,
	handleRobotTrajectory,
} from "./http/robot";
import { sendAgentCommand } from "./agent";
import { handleUpload } from "./http/upload";
import { presignGet, readObject } from "./storage/bucket";

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

// Token-guarded robot ingest → object storage + frames / maps tables.
app.post("/api/robot/frame", (c) => handleRobotFrame(c.req.raw, db));
app.post("/api/robot/frame/:id/analysis", (c) =>
	handleRobotFrameAnalysis(c.req.raw, db, c.req.param("id")),
);
app.post("/api/robot/map", (c) => handleRobotMap(c.req.raw, db));
app.post("/api/robot/splat", (c) => handleRobotSplat(c.req.raw, db));
app.post("/api/robot/trajectory", (c) => handleRobotTrajectory(c.req.raw, db));

// Public read API for room scans, grouped run → positions → angle-sorted images
// (an "array of arrays" of presigned image URLs). Friendly for external callers
// (plain GET + JSON). Presigned with a longer TTL (6h) than the live web UI
// since an API consumer isn't holding a polling page open. `GET /api/scans`
// returns every run; `GET /api/scans/:run` narrows to one.
const SCANS_PRESIGN_TTL = 6 * 60 * 60;
app.get("/api/scans", async (c) =>
	c.json({
		scans: await groupScans(db, presignGet, undefined, SCANS_PRESIGN_TTL),
	}),
);
app.get("/api/scans/:run", async (c) =>
	c.json({
		scans: await groupScans(
			db,
			presignGet,
			c.req.param("run"),
			SCANS_PRESIGN_TTL,
		),
	}),
);

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
		readObject,
			sendAgentCommand,
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
	// Splat uploads (up to 256 MB) stream in over slow links and then get
	// pushed to S3 — well past Bun's 10s default idle timeout, which would
	// otherwise drop the connection mid-upload and surface as a 502 at the
	// gateway. 255s is Bun's max.
	idleTimeout: 255,
	// Allow the full splat size the ingest handler accepts (Bun defaults to
	// 128 MB); the handler enforces the real 256 MB limit and returns 413.
	maxRequestBodySize: 300 * 1024 * 1024,
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
