import type { ApiSession } from "@robomoo/api/server";
import { newId } from "@robomoo/shared";
import { writeImage } from "../storage/bucket";

const MAX_BYTES = 8 * 1024 * 1024; // 8 MB

// Session-guarded multipart upload. Writes the file to the bucket and returns
// the stored object key, which the caller then passes to messages.add.
export async function handleUpload(
  req: Request,
  session: ApiSession | null,
): Promise<Response> {
  if (!session) {
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

  const ext = file.name.includes(".")
    ? file.name.split(".").pop()
    : file.type.split("/").pop();
  const key = `uploads/${newId("img")}.${ext}`;
  await writeImage(key, await file.arrayBuffer(), file.type);

  return Response.json({ key });
}
