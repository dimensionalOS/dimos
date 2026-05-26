import { type Database, frames } from "@robomoo/db";
import { newId } from "@robomoo/shared";
import { env } from "../env";
import { writeImage } from "../storage/bucket";

const MAX_BYTES = 8 * 1024 * 1024; // 8 MB

// Token-guarded frame ingest for the robot. Mirrors the session-guarded
// /api/upload/image handler, but authenticates with a shared bearer secret
// (ROBOT_INGEST_TOKEN) instead of a Better Auth session — the robot has no
// cookie. Writes the JPEG to the bucket and records a row the gallery lists.
export async function handleRobotFrame(
  req: Request,
  db: Database,
): Promise<Response> {
  const token = req.headers.get("authorization")?.replace(/^Bearer\s+/i, "");
  if (!env.ROBOT_INGEST_TOKEN || token !== env.ROBOT_INGEST_TOKEN) {
    return new Response("unauthorized", { status: 401 });
  }

  const form = await req.formData();
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
  const id = newId("frame");
  const ext = file.type.split("/").pop() || "jpg";
  const key = `robot/${id}.${ext}`;
  await writeImage(key, await file.arrayBuffer(), file.type);

  await db.insert(frames).values({
    id,
    imageKey: key,
    note: typeof note === "string" && note.length > 0 ? note : null,
  });

  return Response.json({ key });
}
