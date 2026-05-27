import {
  type Database,
  frameAnalyses,
  frameAnalysisObjects,
  frames,
  maps,
  splats,
  trajectories,
} from "@robomoo/db";
import { newId } from "@robomoo/shared";
import { eq } from "drizzle-orm";
import { env } from "../env";
import { presignGet, writeImage, writeObject } from "../storage/bucket";

const MAX_BYTES = 8 * 1024 * 1024; // 8 MB
const MAX_SPLAT_BYTES = 256 * 1024 * 1024; // 256 MB — splats are large
const SPLAT_EXTS = new Set(["ply", "spz", "splat", "ksplat", "sog"]);
// Per-detection crop/mask PNGs — small (one object each), but a frame can hold
// many. The whole analysis multipart is bounded by Bun's maxRequestBodySize.
const MAX_DETECTION_BYTES = 4 * 1024 * 1024; // 4 MB per crop/mask file

function authed(req: Request): boolean {
  const token = req.headers.get("authorization")?.replace(/^Bearer\s+/i, "");
  return Boolean(env.ROBOT_INGEST_TOKEN) && token === env.ROBOT_INGEST_TOKEN;
}

function num(v: string | File | null): number | null {
  if (typeof v !== "string" || v.length === 0) return null;
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

// Token-guarded frame ingest. Mirrors /api/upload/image but authenticates with
// a shared bearer secret (the robot has no session). Optional pose + label let
// the web place a marker on the map.
export async function handleRobotFrame(
  req: Request,
  db: Database,
): Promise<Response> {
  if (!authed(req)) return new Response("unauthorized", { status: 401 });

  const form = await req.formData().catch(() => null);
  if (!form) {
    return new Response("expected multipart form-data", { status: 400 });
  }
  const file = form.get("file");
  if (!(file instanceof File)) {
    return new Response("expected a 'file' field", { status: 400 });
  }
  if (!file.type.startsWith("image/")) {
    return new Response("only images are allowed", { status: 415 });
  }
  if (file.size > MAX_BYTES) {
    return new Response("file too large (max 8MB)", { status: 413 });
  }

  const note = form.get("note");
  const label = form.get("label");
  const run = form.get("run");
  const position = num(form.get("position"));
  const id = newId("frame");
  const ext = file.type.split("/").pop() || "jpg";
  const key = `robot/${id}.${ext}`;
  await writeImage(key, await file.arrayBuffer(), file.type);

  await db.insert(frames).values({
    id,
    imageKey: key,
    note: typeof note === "string" && note.length > 0 ? note : null,
    label: typeof label === "string" && label.length > 0 ? label : null,
    poseX: num(form.get("poseX")),
    poseY: num(form.get("poseY")),
    embedding: parseEmbedding(form.get("embedding")),
    run: typeof run === "string" && run.length > 0 ? run : null,
    position: position === null ? null : Math.round(position),
    angle: num(form.get("angle")),
  });

  // Fire-and-forget: kick mlxvlm to analyze this frame (Gemma + Falcon). The
  // result lands back via POST /api/robot/frame/:id/analysis. Skipped when
  // MLXVLM_URL is unset so dev/test boots without an analyzer running. Errors
  // are swallowed — analysis is best-effort and the manual "Analyze" buttons
  // remain as a retry path.
  if (env.MLXVLM_URL && env.ROBOT_INGEST_TOKEN) {
    void triggerAnalysis(id, key).catch((e) => {
      console.warn(`[analysis] auto-trigger failed for ${id}:`, e);
    });
  }

  return Response.json({ key });
}

// POST to mlxvlm's /api/analyze-async with a presigned image URL + a callback
// pointing at /api/robot/frame/:id/analysis. mlxvlm runs Gemma + Falcon out of
// band and posts results back; this fn returns as soon as the queue accepts.
async function triggerAnalysis(frameId: string, imageKey: string): Promise<void> {
  if (!env.MLXVLM_URL || !env.ROBOT_INGEST_TOKEN) return;
  // Long TTL — analysis may queue behind other jobs on the Mac.
  const imageUrl = await presignGet(imageKey, 6 * 60 * 60);
  const callbackBase = env.PUBLIC_SERVER_URL ?? null;
  const body = {
    frame_id: frameId,
    image_url: imageUrl,
    callback_base: callbackBase,
    callback_token: env.ROBOT_INGEST_TOKEN,
  };
  const r = await fetch(`${env.MLXVLM_URL.replace(/\/$/, "")}/api/analyze-async`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      // Bypass ngrok-free's HTML interstitial when mlxvlm is exposed through ngrok.
      "ngrok-skip-browser-warning": "true",
    },
    body: JSON.stringify(body),
  });
  if (!r.ok) {
    const text = await r.text().catch(() => "");
    throw new Error(`mlxvlm /api/analyze-async ${r.status}: ${text}`);
  }
}

// Token-guarded analysis-result callback. mlxvlm POSTs this after running
// Gemma + Falcon on a previously-ingested frame. Multipart shape:
//   result : JSON string — { model, description, summary, texts[], objects[] }
//            where each object is { query, label, xy_norm:{x,y}, hw_norm:{w,h},
//            mask_area } — coords already normalized to the frame.
//   crop_<i> : optional PNG of the cropped object (i = index into objects[])
//   mask_<i> : optional single-object alpha mask PNG (i = index into objects[])
// Files are written to s3 and their keys persisted; objects without an
// uploaded crop/mask just store null keys.
export async function handleRobotFrameAnalysis(
  req: Request,
  db: Database,
  frameId: string,
): Promise<Response> {
  if (!authed(req)) return new Response("unauthorized", { status: 401 });

  // Verify the frame exists so a typo'd id doesn't orphan rows.
  const [frame] = await db
    .select({ id: frames.id })
    .from(frames)
    .where(eq(frames.id, frameId))
    .limit(1);
  if (!frame) return new Response("unknown frame", { status: 404 });

  const form = await req.formData().catch(() => null);
  if (!form) return new Response("expected multipart form-data", { status: 400 });
  const resultField = form.get("result");
  if (typeof resultField !== "string") {
    return new Response("missing 'result' JSON field", { status: 400 });
  }

  let parsed: AnalysisPayload;
  try {
    parsed = parseAnalysisPayload(JSON.parse(resultField));
  } catch (e) {
    return new Response(
      `invalid 'result' JSON: ${e instanceof Error ? e.message : "parse error"}`,
      { status: 400 },
    );
  }

  const analysisId = newId("an");
  await db.insert(frameAnalyses).values({
    id: analysisId,
    frameId,
    model: parsed.model,
    description: parsed.description,
    summary: parsed.summary,
    texts: JSON.stringify(parsed.texts),
  });

  // Upload crop_<i> / mask_<i> files first (parallel), then insert object rows.
  const objectRows = await Promise.all(
    parsed.objects.map(async (obj, i) => {
      const cropFile = form.get(`crop_${i}`);
      const maskFile = form.get(`mask_${i}`);
      let cropKey: string | null = null;
      let maskKey: string | null = null;
      if (cropFile instanceof File && cropFile.size > 0) {
        if (cropFile.size > MAX_DETECTION_BYTES) {
          throw new Error(`crop_${i} too large (max 4MB)`);
        }
        cropKey = `analyses/${analysisId}/crop_${i}.png`;
        await writeImage(
          cropKey,
          await cropFile.arrayBuffer(),
          cropFile.type || "image/png",
        );
      }
      if (maskFile instanceof File && maskFile.size > 0) {
        if (maskFile.size > MAX_DETECTION_BYTES) {
          throw new Error(`mask_${i} too large (max 4MB)`);
        }
        maskKey = `analyses/${analysisId}/mask_${i}.png`;
        await writeImage(
          maskKey,
          await maskFile.arrayBuffer(),
          maskFile.type || "image/png",
        );
      }
      return {
        id: newId("ano"),
        analysisId,
        idx: i,
        query: obj.query,
        label: obj.label,
        xyNormX: obj.xyNormX,
        xyNormY: obj.xyNormY,
        hwNormW: obj.hwNormW,
        hwNormH: obj.hwNormH,
        maskArea: obj.maskArea,
        cropKey,
        maskKey,
      };
    }),
  );

  if (objectRows.length > 0) {
    await db.insert(frameAnalysisObjects).values(objectRows);
  }

  return Response.json({ id: analysisId, objectCount: objectRows.length });
}

interface AnalysisObjectInput {
  query: string | null;
  label: string | null;
  xyNormX: number | null;
  xyNormY: number | null;
  hwNormW: number | null;
  hwNormH: number | null;
  maskArea: number | null;
}

interface AnalysisPayload {
  model: string | null;
  description: string | null;
  summary: string | null;
  texts: string[];
  objects: AnalysisObjectInput[];
}

// Tolerant parser — mlxvlm's existing shape uses {xy:{x,y}, hw:{w,h}}; we also
// accept {xy_norm, hw_norm} or flat {x,y,w,h} so the callback contract is easy
// to satisfy without forcing a schema migration on the analyzer.
function parseAnalysisPayload(raw: unknown): AnalysisPayload {
  if (!raw || typeof raw !== "object") throw new Error("not an object");
  const r = raw as Record<string, unknown>;
  const texts = Array.isArray(r.texts)
    ? r.texts.filter((t): t is string => typeof t === "string")
    : [];
  const objectsRaw = Array.isArray(r.objects) ? r.objects : [];
  const objects: AnalysisObjectInput[] = objectsRaw.map((o, i) => {
    if (!o || typeof o !== "object") throw new Error(`objects[${i}] not an object`);
    const x = o as Record<string, unknown>;
    const xy = (x.xy_norm ?? x.xy ?? {}) as Record<string, unknown>;
    const hw = (x.hw_norm ?? x.hw ?? {}) as Record<string, unknown>;
    return {
      query: typeof x.query === "string" ? x.query : null,
      label: typeof x.label === "string" ? x.label : null,
      xyNormX: numOr(xy.x ?? x.x),
      xyNormY: numOr(xy.y ?? x.y),
      hwNormW: numOr(hw.w ?? x.w),
      hwNormH: numOr(hw.h ?? x.h),
      maskArea: numOr(x.mask_area),
    };
  });
  return {
    model: typeof r.model === "string" ? r.model : null,
    description: typeof r.description === "string" ? r.description : null,
    summary: typeof r.summary === "string" ? r.summary : null,
    texts,
    objects,
  };
}

function numOr(v: unknown): number | null {
  if (typeof v === "number" && Number.isFinite(v)) return v;
  if (typeof v === "string") {
    const n = Number(v);
    if (Number.isFinite(n)) return n;
  }
  return null;
}

// Optional CLIP vector sent as a JSON-encoded number[] form field. Anything
// malformed or non-numeric is dropped (the frame is still stored, just
// unsearchable).
function parseEmbedding(v: string | File | null): number[] | null {
  if (typeof v !== "string" || v.length === 0) return null;
  try {
    const arr = JSON.parse(v);
    if (Array.isArray(arr) && arr.every((n) => typeof n === "number")) {
      return arr as number[];
    }
  } catch {
    /* ignore */
  }
  return null;
}

// Token-guarded splat ingest. The reconstruction box (after running COLMAP +
// gaussian-splatting on a batch of frames) POSTs the finished splat file here.
// Accepts .ply/.spz/.splat/.ksplat/.sog; `format` is derived from the field or
// the uploaded filename. `name` is an optional human label.
export async function handleRobotSplat(
  req: Request,
  db: Database,
): Promise<Response> {
  if (!authed(req)) return new Response("unauthorized", { status: 401 });

  const form = await req.formData().catch(() => null);
  if (!form) {
    return new Response("expected multipart form-data", { status: 400 });
  }
  const file = form.get("file");
  if (!(file instanceof File)) {
    return new Response("expected a 'file' field", { status: 400 });
  }
  if (file.size > MAX_SPLAT_BYTES) {
    return new Response("file too large (max 256MB)", { status: 413 });
  }

  const formatField = form.get("format");
  const ext = (
    typeof formatField === "string" && formatField.length > 0
      ? formatField
      : (file.name.split(".").pop() ?? "")
  ).toLowerCase();
  if (!SPLAT_EXTS.has(ext)) {
    return new Response(
      `unsupported splat format '${ext}' (want one of ${[...SPLAT_EXTS].join(", ")})`,
      { status: 415 },
    );
  }

  const nameField = form.get("name");
  const id = newId("splat");
  const key = `splats/${id}.${ext}`;
  await writeObject(
    key,
    await file.arrayBuffer(),
    file.type || "application/octet-stream",
  );

  await db.insert(splats).values({
    id,
    splatKey: key,
    name:
      typeof nameField === "string" && nameField.length > 0 ? nameField : null,
    format: ext,
  });

  return Response.json({ key, id });
}

// Token-guarded map snapshot ingest. The robot renders the dimos global_costmap
// to a PNG and POSTs it with the grid metadata needed to map world → pixels.
export async function handleRobotMap(
  req: Request,
  db: Database,
): Promise<Response> {
  if (!authed(req)) return new Response("unauthorized", { status: 401 });

  const form = await req.formData().catch(() => null);
  if (!form) {
    return new Response("expected multipart form-data", { status: 400 });
  }
  const file = form.get("file");
  if (!(file instanceof File)) {
    return new Response("expected a 'file' field", { status: 400 });
  }
  if (!file.type.startsWith("image/")) {
    return new Response("only images are allowed", { status: 415 });
  }
  if (file.size > MAX_BYTES) {
    return new Response("file too large (max 8MB)", { status: 413 });
  }

  const resolution = num(form.get("resolution"));
  const originX = num(form.get("originX"));
  const originY = num(form.get("originY"));
  const width = num(form.get("width"));
  const height = num(form.get("height"));
  if (
    resolution === null ||
    originX === null ||
    originY === null ||
    width === null ||
    height === null
  ) {
    return new Response(
      "missing map metadata (resolution, originX, originY, width, height)",
      { status: 400 },
    );
  }

  const id = newId("map");
  const key = `robot/${id}.png`;
  await writeImage(key, await file.arrayBuffer(), file.type);

  await db.insert(maps).values({
    id,
    imageKey: key,
    resolution,
    originX,
    originY,
    width: Math.round(width),
    height: Math.round(height),
  });

  return Response.json({ key });
}

// Token-guarded trajectory ingest. The robot POSTs a JSON file — an array of
// {ts, x, y, theta} odometry points — which we store verbatim in object storage
// (same key-indirection as maps) and reference by the newest row.
export async function handleRobotTrajectory(
  req: Request,
  db: Database,
): Promise<Response> {
  if (!authed(req)) return new Response("unauthorized", { status: 401 });

  const form = await req.formData().catch(() => null);
  if (!form) {
    return new Response("expected multipart form-data", { status: 400 });
  }
  const file = form.get("file");
  if (!(file instanceof File)) {
    return new Response("expected a 'file' field", { status: 400 });
  }
  if (file.size > MAX_BYTES) {
    return new Response("file too large (max 8MB)", { status: 413 });
  }

  const id = newId("traj");
  const key = `robot/${id}.json`;
  await writeObject(key, await file.arrayBuffer(), "application/json");

  await db.insert(trajectories).values({ id, pointsKey: key });

  return Response.json({ key });
}
