import type { FileRef } from "../protocol/media.ts";
import { createHash } from "node:crypto";
import { readFile } from "node:fs/promises";

/** The file's bytes, refused when they no longer hash to what was rendered, so a view never shows a changed file as the original. */
export async function readVerified(ref: FileRef): Promise<Buffer> {
  const bytes = await readFile(ref.path);
  const sha256 = createHash("sha256").update(bytes).digest("hex");
  if (sha256 !== ref.sha256) throw new Error(`${ref.path} changed since it was rendered`);
  return bytes;
}
