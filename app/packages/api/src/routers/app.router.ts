import type { RouterClient } from "@orpc/server";
import {
	type Database,
	frames,
	maps,
	messages,
	splats,
	trajectories,
	user,
} from "@robomoo/db";
import {
	createMessageInput,
	type Frame,
	frameSchema,
	type MapSnapshot,
	type Message,
	mapSnapshotSchema,
	messageSchema,
	newId,
	type ScanRun,
	type Splat,
	scanRunSchema,
	splatSchema,
	type Trajectory,
	trajectorySchema,
} from "@robomoo/shared";
import { and, asc, desc, eq, isNotNull } from "drizzle-orm";
import { z } from "zod";
import { protectedProcedure, publicProcedure } from "../api";

// Presigns an object key into a time-limited GET URL — the shape of both the
// oRPC context.presignGet and the server's bucket presignGet.
type PresignFn = (key: string, expiresIn?: number) => Promise<string>;

// Fold the flat frames table into the room-scan structure: run → positions →
// images (angle-sorted), newest run first. Shared by the oRPC `frames.scans`
// procedure and the public REST `/api/scans` route so both stay in lockstep.
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

export const appRouter = {
	messages: { list, add },
	frames: { list: framesList, embedded: framesEmbedded, scans: framesScans },
	map: { latest: mapLatest },
	trajectory: { latest: trajectoryLatest },
	splats: { list: splatsList },
};

export type AppRouter = typeof appRouter;
export type AppRouterClient = RouterClient<typeof appRouter>;
