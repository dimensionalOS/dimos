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
	// CLIP image vector (512-d, normalized) — present only on embedded frames,
	// used by in-browser semantic search. Omitted from the plain gallery list.
	embedding: z.array(z.number()).nullable(),
	createdAt: z.string(),
});
export type Frame = z.infer<typeof frameSchema>;

// A room scan grouped for VR/3D reconstruction. The robot's room_scan sweep tags
// each frame with a `run` (one scan), a `position` (one stop within the run) and
// an `angle` (heading in degrees within that stop's 360° panorama). These schemas
// fold the flat frames table into run → positions → images (angle-sorted), with
// freshly presigned image URLs — the raw S3 key never leaves the server.
export const scanImageSchema = z.object({
	id: z.string(),
	url: z.string(),
	angle: z.number().nullable(),
	// Count of analyses on this frame (latest is fetched via frames.analyses).
	// Lets the gallery render a "analyzed" dot without pulling full payloads.
	analysisCount: z.number().default(0),
});
export type ScanImage = z.infer<typeof scanImageSchema>;

export const scanPositionSchema = z.object({
	position: z.number().nullable(),
	poseX: z.number().nullable(),
	poseY: z.number().nullable(),
	images: z.array(scanImageSchema),
});
export type ScanPosition = z.infer<typeof scanPositionSchema>;

export const scanRunSchema = z.object({
	run: z.string(),
	capturedAt: z.string(),
	positionCount: z.number(),
	imageCount: z.number(),
	positions: z.array(scanPositionSchema),
});
export type ScanRun = z.infer<typeof scanRunSchema>;

// One Falcon-Perception detection inside an analysis. Coordinates are normalized
// to the source image (xy = center, hw = size — multiply by image w/h to render).
// `cropUrl` is a presigned PNG of the cropped object; `maskUrl` is a presigned
// single-object alpha mask PNG. Either can be null when mlxvlm couldn't write a
// usable image (e.g. degenerate bbox, no mask returned).
export const frameAnalysisObjectSchema = z.object({
	id: z.string(),
	idx: z.number(),
	query: z.string().nullable(),
	label: z.string().nullable(),
	xyNormX: z.number().nullable(),
	xyNormY: z.number().nullable(),
	hwNormW: z.number().nullable(),
	hwNormH: z.number().nullable(),
	maskArea: z.number().nullable(),
	cropUrl: z.string().nullable(),
	maskUrl: z.string().nullable(),
});
export type FrameAnalysisObject = z.infer<typeof frameAnalysisObjectSchema>;

// A completed VLM analysis of one frame. `description` is Gemma's scene
// paragraph; `summary` is Gemma's final summary combining scene + Falcon counts;
// `texts` is an array of auxiliary strings worth keeping (initial raw JSON,
// per-query Falcon `raw` lines, etc.). `objects` is the per-detection list with
// presigned crop/mask URLs.
export const frameAnalysisSchema = z.object({
	id: z.string(),
	frameId: z.string(),
	model: z.string().nullable(),
	description: z.string().nullable(),
	summary: z.string().nullable(),
	texts: z.array(z.string()),
	objects: z.array(frameAnalysisObjectSchema),
	createdAt: z.string(),
});
export type FrameAnalysis = z.infer<typeof frameAnalysisSchema>;

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

// The latest 2D occupancy map snapshot (the dimos global_costmap). The robot
// uploads a value-preserving grayscale PNG (free=0, occupied=1..100,
// unknown=255); the server inlines it as a same-origin `imageDataUri` so the
// canvas can read raw cell values back (no cross-origin taint) and apply its own
// colormap. `resolution` (m/cell) + `origin` place world coordinates onto the
// grid: col = (x - originX) / resolution, row = (y - originY) / resolution.
export const mapSnapshotSchema = z.object({
	imageDataUri: z.string(),
	resolution: z.number(),
	originX: z.number(),
	originY: z.number(),
	width: z.number(),
	height: z.number(),
	createdAt: z.string(),
});
export type MapSnapshot = z.infer<typeof mapSnapshotSchema>;

// Robot odometry trajectory: an ordered path of world poses. `theta` is the
// heading (yaw, radians). The newest snapshot is inlined (the path is small) so
// the web can draw a polyline + a heading marker at the latest point without a
// second cross-origin fetch.
export const trajectoryPointSchema = z.object({
	ts: z.number(),
	x: z.number(),
	y: z.number(),
	theta: z.number(),
});
export type TrajectoryPoint = z.infer<typeof trajectoryPointSchema>;

export const trajectorySchema = z.object({
	points: z.array(trajectoryPointSchema),
	createdAt: z.string(),
});
export type Trajectory = z.infer<typeof trajectorySchema>;
