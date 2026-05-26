import { type Database, frames, maps, splats } from "@robomoo/db";
import { newId } from "@robomoo/shared";
import { env } from "../env";
import { writeImage, writeObject } from "../storage/bucket";

const MAX_BYTES = 8 * 1024 * 1024; // 8 MB
const MAX_SPLAT_BYTES = 256 * 1024 * 1024; // 256 MB — splats are large
const SPLAT_EXTS = new Set(["ply", "spz", "splat", "ksplat", "sog"]);

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
  });

  return Response.json({ key });
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
