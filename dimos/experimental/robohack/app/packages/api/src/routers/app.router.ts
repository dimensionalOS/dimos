import { ORPCError, type RouterClient } from "@orpc/server";
import {
	agents,
	type Database,
	frameAnalyses,
	frameAnalysisObjects,
	frames,
	jobs,
	maps,
	messages,
	settings,
	splats,
	trajectories,
	user,
} from "@robomoo/db";
import {
	type Agent,
	type AgentService,
	type AgentStats,
	type AgentStatus,
	agentSchema,
	agentStatsSchema,
	commandInput,
	createJobInput,
	createMessageInput,
	type Frame,
	type FrameAnalysis,
	type FrameAnalysisObject,
	frameAnalysisSchema,
	frameSchema,
	type Job,
	type JobStatus,
	jobSchema,
	type MapSnapshot,
	type Message,
	mapSnapshotSchema,
	messageSchema,
	newId,
	type RobotSettings,
	robotSettingsSchema,
	type ScanRun,
	type ScanRunHeader,
	setAgentOnchainInput,
	setRobotUrlInput,
	type Splat,
	scanRunHeaderSchema,
	scanRunSchema,
	splatSchema,
	type Trajectory,
	trajectorySchema,
} from "@robomoo/shared";
import { and, asc, desc, eq, gt, inArray, isNotNull, sql } from "drizzle-orm";
import { z } from "zod";
import { protectedProcedure, publicProcedure } from "../api";

// Presigns an object key into a time-limited GET URL — the shape of both the
// oRPC context.presignGet and the server's bucket presignGet.
type PresignFn = (key: string, expiresIn?: number) => Promise<string>;

// Fold the flat frames table into the room-scan structure: run → positions →
// images (angle-sorted), newest run first. Shared by the oRPC `frames.scans`
// procedure and the public REST `/api/scans` route so both stay in lockstep.
// Raw `sql<Date>` aggregates (max/min) come back from node-postgres as strings,
// not Dates — drizzle only maps real columns, so the `<Date>` cast is a
// compile-time-only lie. Coerce to a real Date (the tz-naive value is UTC,
// matching how drizzle reads timestamp columns) before .toISOString()/.getTime().
function pgTs(v: Date | string): Date {
	return v instanceof Date ? v : new Date(`${String(v).replace(" ", "T")}Z`);
}

export async function groupScans(
	db: Database,
	presignGet: PresignFn,
	run?: string,
	expiresIn = 3600,
): Promise<ScanRun[]> {
	const rows = await db
		.select()
		.from(frames)
		.where(
			run
				? and(isNotNull(frames.run), eq(frames.run, run))
				: isNotNull(frames.run),
		)
		.orderBy(asc(frames.run), asc(frames.position), asc(frames.angle))
		.limit(5000);

	// Presign every image up front (in parallel) so the fold below is sync.
	const urls = new Map<string, string>();
	await Promise.all(
		rows.map(async (r) => {
			urls.set(r.id, await presignGet(r.imageKey, expiresIn));
		}),
	);

	// Per-frame analysis count (latest is fetched separately via frames.analyses).
	// One grouped query — keeps the per-row work O(1).
	const frameIds = rows.map((r) => r.id);
	const analysisCounts = new Map<string, number>();
	if (frameIds.length > 0) {
		const countRows = await db
			.select({
				frameId: frameAnalyses.frameId,
				count: sql<number>`count(*)::int`,
			})
			.from(frameAnalyses)
			.where(inArray(frameAnalyses.frameId, frameIds))
			.groupBy(frameAnalyses.frameId);
		for (const c of countRows) analysisCounts.set(c.frameId, Number(c.count));
	}

	// run → (position → accumulator). Maps preserve insertion order, and rows are
	// already sorted by run/position/angle, so the nested arrays come out ordered.
	const runs = new Map<
		string,
		{
			capturedAt: Date;
			positions: Map<
				number,
				{
					poseX: number | null;
					poseY: number | null;
					images: ScanRun["positions"][number]["images"];
				}
			>;
		}
	>();

	for (const r of rows) {
		if (r.run === null) continue;
		let runEntry = runs.get(r.run);
		if (!runEntry) {
			runEntry = { capturedAt: r.createdAt, positions: new Map() };
			runs.set(r.run, runEntry);
		}
		// Track the run's most recent capture for sorting/display.
		if (r.createdAt > runEntry.capturedAt) runEntry.capturedAt = r.createdAt;

		// Null position frames (shouldn't happen for room_scan) bucket under -1.
		const posKey = r.position ?? -1;
		let posEntry = runEntry.positions.get(posKey);
		if (!posEntry) {
			posEntry = { poseX: r.poseX, poseY: r.poseY, images: [] };
			runEntry.positions.set(posKey, posEntry);
		}
		posEntry.images.push({
			id: r.id,
			url: urls.get(r.id) ?? "",
			angle: r.angle,
			analysisCount: analysisCounts.get(r.id) ?? 0,
		});
	}

	return [...runs.entries()]
		.map(([run, entry]) => {
			const positions = [...entry.positions.entries()].map(([position, p]) => ({
				position: position === -1 ? null : position,
				poseX: p.poseX,
				poseY: p.poseY,
				images: p.images,
			}));
			return {
				run,
				capturedAt: entry.capturedAt.toISOString(),
				positionCount: positions.length,
				imageCount: positions.reduce((n, p) => n + p.images.length, 0),
				positions,
			};
		})
		.sort((a, b) => b.capturedAt.localeCompare(a.capturedAt));
}

const list = publicProcedure
	.output(z.array(messageSchema))
	.handler(async ({ context }): Promise<Message[]> => {
		const rows = await context.db
			.select({
				id: messages.id,
				body: messages.body,
				imageKey: messages.imageKey,
				createdAt: messages.createdAt,
				authorName: user.name,
			})
			.from(messages)
			.innerJoin(user, eq(messages.authorId, user.id))
			.orderBy(desc(messages.createdAt))
			.limit(50);

		return Promise.all(
			rows.map(async (r) => ({
				id: r.id,
				body: r.body,
				imageUrl: r.imageKey ? await context.presignGet(r.imageKey) : null,
				authorName: r.authorName,
				createdAt: r.createdAt.toISOString(),
			})),
		);
	});

const add = protectedProcedure
	.input(createMessageInput)
	.output(messageSchema)
	.handler(async ({ context, input }): Promise<Message> => {
		const [row] = await context.db
			.insert(messages)
			.values({
				id: newId("msg"),
				body: input.body,
				imageKey: input.imageKey,
				authorId: context.session.user.id,
			})
			.returning();

		if (!row) {
			throw new Error("insert returned no row");
		}

		return {
			id: row.id,
			body: row.body,
			imageUrl: row.imageKey ? await context.presignGet(row.imageKey) : null,
			authorName: context.session.user.name,
			createdAt: row.createdAt.toISOString(),
		};
	});

// Robot frame gallery — public list, newest first, with presigned image URLs.
const framesList = publicProcedure
	.output(z.array(frameSchema))
	.handler(async ({ context }): Promise<Frame[]> => {
		const rows = await context.db
			.select()
			.from(frames)
			.orderBy(desc(frames.createdAt))
			.limit(100);

		return Promise.all(
			rows.map(async (r) => ({
				id: r.id,
				imageUrl: await context.presignGet(r.imageKey),
				note: r.note,
				label: r.label,
				poseX: r.poseX,
				poseY: r.poseY,
				embedding: null, // gallery doesn't need vectors — keep the payload light
				createdAt: r.createdAt.toISOString(),
			})),
		);
	});

// Embedded frames only (have a CLIP vector) — fuels in-browser semantic search.
// Ships the vectors so cosine ranking happens entirely client-side.
const framesEmbedded = publicProcedure
	.output(z.array(frameSchema))
	.handler(async ({ context }): Promise<Frame[]> => {
		const rows = await context.db
			.select()
			.from(frames)
			.where(isNotNull(frames.embedding))
			.orderBy(desc(frames.createdAt))
			.limit(2000);

		return Promise.all(
			rows.map(async (r) => ({
				id: r.id,
				imageUrl: await context.presignGet(r.imageKey),
				note: r.note,
				label: r.label,
				poseX: r.poseX,
				poseY: r.poseY,
				embedding: r.embedding,
				createdAt: r.createdAt.toISOString(),
			})),
		);
	});

// All VLM analyses for the given frame ids (latest-first), with presigned
// crop/mask URLs per detection. Web calls this on demand (only when a lightbox
// opens or a run is expanded with analyses panel), so the gallery list payload
// stays small. Bounded to 50 frame ids per call to keep the join tractable.
const framesAnalyses = publicProcedure
	.input(z.object({ frameIds: z.array(z.string()).min(1).max(50) }))
	.output(z.array(frameAnalysisSchema))
	.handler(async ({ context, input }): Promise<FrameAnalysis[]> => {
		const analyses = await context.db
			.select()
			.from(frameAnalyses)
			.where(inArray(frameAnalyses.frameId, input.frameIds))
			.orderBy(desc(frameAnalyses.createdAt))
			.limit(500);

		if (analyses.length === 0) return [];

		const analysisIds = analyses.map((a) => a.id);
		const objectRows = await context.db
			.select()
			.from(frameAnalysisObjects)
			.where(inArray(frameAnalysisObjects.analysisId, analysisIds))
			.orderBy(asc(frameAnalysisObjects.idx));

		// Presign every crop/mask URL up front, in parallel.
		const presignedCrops = new Map<string, string>();
		const presignedMasks = new Map<string, string>();
		await Promise.all(
			objectRows.flatMap((o) => {
				const tasks: Promise<void>[] = [];
				if (o.cropKey) {
					tasks.push(
						context
							.presignGet(o.cropKey)
							.then((u) => void presignedCrops.set(o.id, u)),
					);
				}
				if (o.maskKey) {
					tasks.push(
						context
							.presignGet(o.maskKey)
							.then((u) => void presignedMasks.set(o.id, u)),
					);
				}
				return tasks;
			}),
		);

		const objectsByAnalysis = new Map<string, FrameAnalysisObject[]>();
		for (const o of objectRows) {
			const list = objectsByAnalysis.get(o.analysisId) ?? [];
			list.push({
				id: o.id,
				idx: o.idx,
				query: o.query,
				label: o.label,
				xyNormX: o.xyNormX,
				xyNormY: o.xyNormY,
				hwNormW: o.hwNormW,
				hwNormH: o.hwNormH,
				maskArea: o.maskArea,
				cropUrl: o.cropKey ? (presignedCrops.get(o.id) ?? null) : null,
				maskUrl: o.maskKey ? (presignedMasks.get(o.id) ?? null) : null,
			});
			objectsByAnalysis.set(o.analysisId, list);
		}

		return analyses.map((a) => {
			let texts: string[] = [];
			if (a.texts) {
				try {
					const parsed = JSON.parse(a.texts);
					if (Array.isArray(parsed)) {
						texts = parsed.filter((t): t is string => typeof t === "string");
					}
				} catch {
					// Stored value isn't valid JSON — treat as a single text line.
					texts = [a.texts];
				}
			}
			return {
				id: a.id,
				frameId: a.frameId,
				model: a.model,
				description: a.description,
				summary: a.summary,
				texts,
				objects: objectsByAnalysis.get(a.id) ?? [],
				createdAt: a.createdAt.toISOString(),
			};
		});
	});

// Cheap aggregate of every run: counts + timestamps, no per-image rows and no
// presigned URLs. Fuels the collapsed accordion on /scans — the page polls
// this every 5s so it must stay O(runs), not O(images). `latestAnalysisAt`
// doubles as an ETag the client uses to know when to refetch a given run's
// full details (via `frames.scans({ run })`).
const framesScansHeaders = publicProcedure
	.output(z.array(scanRunHeaderSchema))
	.handler(async ({ context }): Promise<ScanRunHeader[]> => {
		// One row per run. Sort happens in JS — keeps the SQL simple and avoids
		// any drizzle ordering-on-aggregate quirks.
		let headerRows: {
			run: string | null;
			capturedAt: Date;
			imageCount: number;
			positionCount: number;
		}[];
		try {
			headerRows = await context.db
				.select({
					run: frames.run,
					capturedAt: sql<Date>`max(${frames.createdAt})`,
					imageCount: sql<number>`count(*)::int`,
					positionCount: sql<number>`count(distinct ${frames.position})::int`,
				})
				.from(frames)
				.where(isNotNull(frames.run))
				.groupBy(frames.run);
		} catch (e) {
			console.error("[scansHeaders] headers query failed", e);
			throw e;
		}

		// Per-run analysis count + most-recent analysis ts (the ETag). Wrapped in
		// try/catch — if the frame_analyses migration hasn't applied yet (fresh
		// deploy mid-rollout), we still want the headers list to render. The
		// run rows just show 0 analyses until the table exists.
		let analysisRows: {
			run: string | null;
			analysisCount: number;
			latestAnalysisAt: Date | null;
		}[] = [];
		try {
			analysisRows = await context.db
				.select({
					run: frames.run,
					analysisCount: sql<number>`count(*)::int`,
					latestAnalysisAt: sql<Date>`max(${frameAnalyses.createdAt})`,
				})
				.from(frameAnalyses)
				.innerJoin(frames, eq(frameAnalyses.frameId, frames.id))
				.where(isNotNull(frames.run))
				.groupBy(frames.run);
		} catch (e) {
			console.warn(
				"[scansHeaders] analyses query failed (table missing?) — continuing with zero counts",
				e,
			);
		}

		const analysisByRun = new Map(
			analysisRows
				.filter((r): r is typeof r & { run: string } => r.run !== null)
				.map((r) => [r.run, r]),
		);

		return headerRows
			.filter((r): r is typeof r & { run: string } => r.run !== null)
			.map((r) => {
				const a = analysisByRun.get(r.run);
				return {
					run: r.run,
					capturedAt: pgTs(r.capturedAt).toISOString(),
					positionCount: Number(r.positionCount),
					imageCount: Number(r.imageCount),
					analysisCount: a ? Number(a.analysisCount) : 0,
					latestAnalysisAt: a?.latestAnalysisAt
						? pgTs(a.latestAnalysisAt).toISOString()
						: null,
				};
			})
			.sort((a, b) => b.capturedAt.localeCompare(a.capturedAt));
	});

// Room scans grouped run → positions → angle-sorted images. Public; fuels the
// /scans browse page. Optional `run` narrows to a single scan.
const framesScans = publicProcedure
	.input(z.object({ run: z.string().optional() }).optional())
	.output(z.array(scanRunSchema))
	.handler(async ({ context, input }): Promise<ScanRun[]> => {
		return groupScans(context.db, context.presignGet, input?.run);
	});

// Newest map snapshot (or null if none yet) with a presigned PNG URL + the grid
// metadata the web needs to place world coordinates onto the image.
const mapLatest = publicProcedure
	.output(mapSnapshotSchema.nullable())
	.handler(async ({ context }): Promise<MapSnapshot | null> => {
		const [row] = await context.db
			.select()
			.from(maps)
			.orderBy(desc(maps.createdAt))
			.limit(1);
		if (!row) return null;
		const bytes = await context.readObject(row.imageKey);
		const b64 = Buffer.from(bytes).toString("base64");
		return {
			imageDataUri: `data:image/png;base64,${b64}`,
			resolution: row.resolution,
			originX: row.originX,
			originY: row.originY,
			width: row.width,
			height: row.height,
			createdAt: row.createdAt.toISOString(),
		};
	});

// Newest odometry trajectory (or null). The path JSON lives in object storage;
// we read + inline it (it's small) so the web gets the points directly.
const trajectoryLatest = publicProcedure
	.output(trajectorySchema.nullable())
	.handler(async ({ context }): Promise<Trajectory | null> => {
		const [row] = await context.db
			.select()
			.from(trajectories)
			.orderBy(desc(trajectories.createdAt))
			.limit(1);
		if (!row) return null;
		const bytes = await context.readObject(row.pointsKey);
		const points = JSON.parse(Buffer.from(bytes).toString("utf8"));
		return { points, createdAt: row.createdAt.toISOString() };
	});

// 3D Gaussian splats — public list, newest first. Presigned with a long TTL
// (24h) since the files are large and slow to stream in the browser viewer.
const splatsList = publicProcedure
	.output(z.array(splatSchema))
	.handler(async ({ context }): Promise<Splat[]> => {
		const rows = await context.db
			.select()
			.from(splats)
			.orderBy(desc(splats.createdAt))
			.limit(100);

		return Promise.all(
			rows.map(async (r) => ({
				id: r.id,
				splatUrl: await context.presignGet(r.splatKey, 86400),
				name: r.name,
				format: r.format,
				createdAt: r.createdAt.toISOString(),
			})),
		);
	});

// Forward a natural-language command to the robot's agent (open/demo mode). The
// dimos endpoint URL + token live server-side (context.sendAgentCommand); the
// browser only ever sees this RPC.
const commandSend = publicProcedure
	.input(commandInput)
	.output(z.object({ ok: z.boolean() }))
	.handler(async ({ context, input }): Promise<{ ok: boolean }> => {
		try {
			await context.sendAgentCommand(input.text);
		} catch (e) {
			// Surface the real reason (not configured / endpoint error) to the UI
			// instead of a generic 500.
			throw new ORPCError("BAD_REQUEST", {
				message: e instanceof Error ? e.message : "failed to send command",
			});
		}
		return { ok: true };
	});

// ─── Robot connection settings ──────────────────────────────────────────────
// Single-row config for the dimos agent endpoint. The DB override (set here)
// wins over the server's DIMOS_AGENT_URL env; the bearer token stays in the env.
const SETTINGS_SINGLETON = "singleton";

async function readRobotSettings(context: {
	db: Database;
	agentEnvUrl: string | null;
	agentTokenConfigured: boolean;
}): Promise<RobotSettings> {
	const [row] = await context.db
		.select()
		.from(settings)
		.where(eq(settings.id, SETTINGS_SINGLETON));
	const url = row?.dimosAgentUrl ?? null;
	const effectiveUrl = url ?? context.agentEnvUrl;
	const source = url ? "db" : context.agentEnvUrl ? "env" : "unset";
	return {
		url,
		effectiveUrl,
		source,
		tokenConfigured: context.agentTokenConfigured,
	};
}

const settingsGet = publicProcedure
	.output(robotSettingsSchema)
	.handler(({ context }): Promise<RobotSettings> => readRobotSettings(context));

const settingsSetRobotUrl = publicProcedure
	.input(setRobotUrlInput)
	.output(robotSettingsSchema)
	.handler(async ({ context, input }): Promise<RobotSettings> => {
		await context.db
			.insert(settings)
			.values({
				id: SETTINGS_SINGLETON,
				dimosAgentUrl: input.url,
				updatedAt: new Date(),
			})
			.onConflictDoUpdate({
				target: settings.id,
				set: { dimosAgentUrl: input.url, updatedAt: new Date() },
			});
		return readRobotSettings(context);
	});

// ─── Agent marketplace (the ERC-8004 registry surface) ──────────────────────

async function toAgent(
	row: typeof agents.$inferSelect,
	presignGet: PresignFn,
): Promise<Agent> {
	return {
		id: row.id,
		slug: row.slug,
		name: row.name,
		tagline: row.tagline,
		description: row.description,
		avatarUrl: row.avatarKey ? await presignGet(row.avatarKey) : null,
		emoji: row.emoji,
		services: row.services as AgentService[],
		basePriceUsd: row.basePriceUsd,
		isReal: row.isReal,
		status: row.status as AgentStatus,
		chain: row.chain,
		agentId: row.agentId,
		agentWallet: row.agentWallet,
		registerTx: row.registerTx,
		capabilities: row.capabilities,
		createdAt: row.createdAt.toISOString(),
	};
}

function toJob(row: typeof jobs.$inferSelect): Job {
	return {
		id: row.id,
		agentSlug: row.agentSlug,
		requesterAddr: row.requesterAddr,
		requesterUserId: row.requesterUserId,
		service: row.service,
		status: row.status as JobStatus,
		priceUsd: row.priceUsd,
		paid: row.paid,
		paymentMode: row.paymentMode,
		paymentTx: row.paymentTx,
		command: row.command,
		run: row.run,
		splatId: row.splatId,
		rating: row.rating,
		feedbackTx: row.feedbackTx,
		createdAt: row.createdAt.toISOString(),
		dispatchedAt: row.dispatchedAt ? row.dispatchedAt.toISOString() : null,
		completedAt: row.completedAt ? row.completedAt.toISOString() : null,
	};
}

// Claim-latest reconciliation: pin the freshest scan run (and then a splat
// deliverable) produced after the job was dispatched. Mutates the job row on
// read — fine for a single-dog demo. A run is claimed when its earliest frame
// landed at/after dispatch; a splat is attached once a run is pinned.
async function reconcileJob(
	db: Database,
	row: typeof jobs.$inferSelect,
): Promise<typeof jobs.$inferSelect> {
	let job = row;

	if (
		(job.status === "scanning" || job.status === "dispatched") &&
		!job.run &&
		job.dispatchedAt
	) {
		const dispatchedAt = job.dispatchedAt;
		const runRows = await db
			.select({
				run: frames.run,
				firstAt: sql<Date>`min(${frames.createdAt})`,
			})
			.from(frames)
			.where(isNotNull(frames.run))
			.groupBy(frames.run);
		const claimed = runRows
			.filter(
				(r): r is { run: string; firstAt: Date } =>
					r.run !== null && pgTs(r.firstAt) >= dispatchedAt,
			)
			.sort((a, b) => pgTs(b.firstAt).getTime() - pgTs(a.firstAt).getTime())[0];
		if (claimed) {
			const [updated] = await db
				.update(jobs)
				.set({ run: claimed.run, status: "scanning" })
				.where(eq(jobs.id, job.id))
				.returning();
			if (updated) job = updated;
		}
	}

	if (job.run && !job.splatId && job.dispatchedAt) {
		const [s] = await db
			.select()
			.from(splats)
			.where(gt(splats.createdAt, job.dispatchedAt))
			.orderBy(desc(splats.createdAt))
			.limit(1);
		if (s) {
			const [updated] = await db
				.update(jobs)
				.set({ splatId: s.id, status: "done", completedAt: new Date() })
				.where(eq(jobs.id, job.id))
				.returning();
			if (updated) job = updated;
		}
	}

	return job;
}

const agentsList = publicProcedure
	.output(z.array(agentSchema))
	.handler(async ({ context }): Promise<Agent[]> => {
		const rows = await context.db
			.select()
			.from(agents)
			.orderBy(desc(agents.isReal), asc(agents.createdAt));
		return Promise.all(rows.map((r) => toAgent(r, context.presignGet)));
	});

const agentsGet = publicProcedure
	.input(z.object({ slug: z.string() }))
	.output(agentSchema.nullable())
	.handler(async ({ context, input }): Promise<Agent | null> => {
		const [row] = await context.db
			.select()
			.from(agents)
			.where(eq(agents.slug, input.slug))
			.limit(1);
		return row ? await toAgent(row, context.presignGet) : null;
	});

// Persist the result of a browser-wallet ERC-8004 register() call. Public for
// the demo (no operator auth) — anyone who registers an agent can bind its id.
const agentsSetOnchain = publicProcedure
	.input(setAgentOnchainInput)
	.output(agentSchema)
	.handler(async ({ context, input }): Promise<Agent> => {
		const [row] = await context.db
			.update(agents)
			.set({
				agentId: input.agentId,
				agentWallet: input.agentWallet,
				registerTx: input.registerTx,
			})
			.where(eq(agents.slug, input.slug))
			.returning();
		if (!row) throw new Error("unknown agent");
		return toAgent(row, context.presignGet);
	});

// Job-derived reputation fallback (used when no on-chain summary is available).
const agentsStats = publicProcedure
	.input(z.object({ slug: z.string() }))
	.output(agentStatsSchema)
	.handler(async ({ context, input }): Promise<AgentStats> => {
		const rows = await context.db
			.select({ status: jobs.status, rating: jobs.rating })
			.from(jobs)
			.where(eq(jobs.agentSlug, input.slug));
		const completed = rows.filter((r) => r.status === "done").length;
		const ratings = rows
			.map((r) => r.rating)
			.filter((n): n is number => typeof n === "number");
		const avgRating =
			ratings.length > 0
				? ratings.reduce((a, b) => a + b, 0) / ratings.length
				: null;
		return { completed, rated: ratings.length, avgRating };
	});

const jobsCreate = publicProcedure
	.input(createJobInput)
	.output(jobSchema)
	.handler(async ({ context, input }): Promise<Job> => {
		const [agent] = await context.db
			.select()
			.from(agents)
			.where(eq(agents.slug, input.agentSlug))
			.limit(1);
		if (!agent) throw new Error("unknown agent");
		const svc = (agent.services as AgentService[]).find(
			(s) => s.key === input.service,
		);
		if (!svc) throw new Error("unknown service");
		const [row] = await context.db
			.insert(jobs)
			.values({
				id: newId("job"),
				agentSlug: input.agentSlug,
				service: input.service,
				requesterAddr: input.requesterAddr,
				priceUsd: svc.priceUsd,
				status: "booked",
			})
			.returning();
		if (!row) throw new Error("insert returned no row");
		return toJob(row);
	});

const jobsPay = publicProcedure
	.input(
		z.object({
			id: z.string(),
			paymentMode: z.enum(["mock", "x402"]).default("mock"),
			paymentTx: z.string().nullable().default(null),
		}),
	)
	.output(jobSchema)
	.handler(async ({ context, input }): Promise<Job> => {
		const [row] = await context.db
			.update(jobs)
			.set({
				paid: true,
				paymentMode: input.paymentMode,
				paymentTx: input.paymentTx,
			})
			.where(eq(jobs.id, input.id))
			.returning();
		if (!row) throw new Error("unknown job");
		return toJob(row);
	});

const jobsDispatch = publicProcedure
	.input(z.object({ id: z.string(), command: z.string().min(1).max(500) }))
	.output(jobSchema)
	.handler(async ({ context, input }): Promise<Job> => {
		const [row] = await context.db
			.update(jobs)
			.set({
				status: "scanning",
				dispatchedAt: new Date(),
				command: input.command,
			})
			.where(eq(jobs.id, input.id))
			.returning();
		if (!row) throw new Error("unknown job");
		// Best-effort forward to the live robot. The demo proceeds even with no
		// agent endpoint configured (sendAgentCommand throws → swallowed).
		try {
			await context.sendAgentCommand(input.command);
		} catch (e) {
			console.warn(
				`[jobs.dispatch] agent command not sent for ${row.id}:`,
				e instanceof Error ? e.message : e,
			);
		}
		return toJob(row);
	});

const jobsGet = publicProcedure
	.input(z.object({ id: z.string() }))
	.output(jobSchema.nullable())
	.handler(async ({ context, input }): Promise<Job | null> => {
		const [row] = await context.db
			.select()
			.from(jobs)
			.where(eq(jobs.id, input.id))
			.limit(1);
		if (!row) return null;
		return toJob(await reconcileJob(context.db, row));
	});

const jobsList = publicProcedure
	.input(z.object({ address: z.string().nullable().default(null) }).optional())
	.output(z.array(jobSchema))
	.handler(async ({ context, input }): Promise<Job[]> => {
		const addr = input?.address ?? null;
		const rows = await context.db
			.select()
			.from(jobs)
			.where(addr ? eq(jobs.requesterAddr, addr) : sql`1=1`)
			.orderBy(desc(jobs.createdAt))
			.limit(50);
		const reconciled = await Promise.all(
			rows.map((r) =>
				r.status === "scanning" || r.status === "dispatched"
					? reconcileJob(context.db, r)
					: Promise.resolve(r),
			),
		);
		return reconciled.map(toJob);
	});

const jobsAttachRun = publicProcedure
	.input(z.object({ id: z.string(), run: z.string() }))
	.output(jobSchema)
	.handler(async ({ context, input }): Promise<Job> => {
		const [row] = await context.db
			.update(jobs)
			.set({ run: input.run, status: "scanning" })
			.where(eq(jobs.id, input.id))
			.returning();
		if (!row) throw new Error("unknown job");
		return toJob(row);
	});

const jobsComplete = publicProcedure
	.input(z.object({ id: z.string(), splatId: z.string().nullable().default(null) }))
	.output(jobSchema)
	.handler(async ({ context, input }): Promise<Job> => {
		const [row] = await context.db
			.update(jobs)
			.set({
				status: "done",
				completedAt: new Date(),
				...(input.splatId ? { splatId: input.splatId } : {}),
			})
			.where(eq(jobs.id, input.id))
			.returning();
		if (!row) throw new Error("unknown job");
		return toJob(row);
	});

const jobsSetFeedback = publicProcedure
	.input(
		z.object({
			id: z.string(),
			rating: z.number().int().min(1).max(5),
			feedbackTx: z.string().nullable().default(null),
		}),
	)
	.output(jobSchema)
	.handler(async ({ context, input }): Promise<Job> => {
		const [row] = await context.db
			.update(jobs)
			.set({ rating: input.rating, feedbackTx: input.feedbackTx })
			.where(eq(jobs.id, input.id))
			.returning();
		if (!row) throw new Error("unknown job");
		return toJob(row);
	});

export const appRouter = {
	messages: { list, add },
	frames: {
		list: framesList,
		embedded: framesEmbedded,
		scans: framesScans,
		scansHeaders: framesScansHeaders,
		analyses: framesAnalyses,
	},
	map: { latest: mapLatest },
	trajectory: { latest: trajectoryLatest },
	splats: { list: splatsList },
	commands: { send: commandSend },
	settings: { get: settingsGet, setRobotUrl: settingsSetRobotUrl },
	agents: {
		list: agentsList,
		get: agentsGet,
		setOnchain: agentsSetOnchain,
		stats: agentsStats,
	},
	jobs: {
		create: jobsCreate,
		pay: jobsPay,
		dispatch: jobsDispatch,
		get: jobsGet,
		list: jobsList,
		attachRun: jobsAttachRun,
		complete: jobsComplete,
		setFeedback: jobsSetFeedback,
	},
};

export type AppRouter = typeof appRouter;
export type AppRouterClient = RouterClient<typeof appRouter>;
