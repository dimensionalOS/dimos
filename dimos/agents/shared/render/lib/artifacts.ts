import type { FileRef, Sha256 } from "../../protocol/media.ts";
import { createHash, randomUUID } from "node:crypto";
import { join } from "node:path";
import { mkdir, readFile, readdir, rename, stat, unlink, utimes, writeFile } from "node:fs/promises";

/** A source read once, size-capped and hashed; `json` is undefined unless the bytes parse as JSON. */
export type SourceFile = {
  readonly path: string;
  readonly sha256: Sha256;
  readonly bytes: Buffer;
  readonly json: unknown;
};

function sha256Of(bytes: Buffer): Sha256 {
  return createHash("sha256").update(bytes).digest("hex") as Sha256;
}

function parseJson(bytes: Buffer): unknown {
  const first = bytes.find((byte) => byte !== 0x20 && byte !== 0x0a && byte !== 0x0d && byte !== 0x09);
  if (first !== 0x7b && first !== 0x5b) return undefined;
  try {
    return JSON.parse(bytes.toString("utf8"));
  } catch {
    return undefined;
  }
}

/**
 * Reads a regular file of at most `maxBytes`, hashes the bytes, and throws when an expected hash differs. Moved from
 * dimcode artifacts.ts.
 */
export async function readSource(path: string, maxBytes: number, expected: Sha256 | null): Promise<SourceFile> {
  const info = await stat(path);
  // A FIFO or a device has size 0 and may never end, so only a regular file's size bounds the read.
  if (!info.isFile()) throw new Error("not a regular file");
  if (info.size > maxBytes) throw new Error(`${info.size} bytes, over the ${maxBytes}-byte limit`);
  const bytes = await readFile(path);
  const sha256 = sha256Of(bytes);
  if (expected !== null && expected !== sha256) {
    throw new Error(`changed since it was exported: sha256 was ${expected}, is ${sha256}`);
  }
  return { path, sha256, bytes, json: parseJson(bytes) };
}

/** What a renderer stores its previews in during one render. */
export type PreviewCache = { put(bytes: Buffer, extension: "png" | "gif"): Promise<FileRef> };

/** Content-addressed PNG and GIF previews: directory 0700, files 0600, oldest pruned past `maxBytes`. */
export class RenderCache {
  private readonly dir: string;
  private readonly maxBytes: number;
  /** The names each unfinished render has put; prune deletes none of them. */
  private readonly pinned = new Set<ReadonlySet<string>>();

  constructor(dir: string, maxBytes: number) {
    this.dir = dir;
    this.maxBytes = maxBytes;
  }

  /** Runs `render` with puts that stay until it settles, so a render past `maxBytes` keeps its own earlier previews. */
  async pinning<T>(render: (cache: PreviewCache) => Promise<T>): Promise<T> {
    const names = new Set<string>();
    this.pinned.add(names);
    try {
      return await render({ put: (bytes, extension) => this.put(bytes, extension, names) });
    } finally {
      this.pinned.delete(names);
    }
  }

  private async put(bytes: Buffer, extension: "png" | "gif", names: Set<string>): Promise<FileRef> {
    await mkdir(this.dir, { recursive: true, mode: 0o700 });
    const sha256 = sha256Of(bytes);
    const name = `${sha256}.${extension}`;
    const path = join(this.dir, name);
    names.add(name);
    const now = new Date();
    try {
      await utimes(path, now, now);
    } catch {
      // The gateway and a terminal's /inspect share the directory, so a reader never sees a half-written file.
      const partial = join(this.dir, `.${name}.${randomUUID()}`);
      await writeFile(partial, bytes, { mode: 0o600 });
      await rename(partial, path);
    }
    await this.prune();
    return { path, sha256 };
  }

  private async prune(): Promise<void> {
    const names = (await readdir(this.dir)).filter((name) => /^[0-9a-f]{64}\.(png|gif)$/.test(name));
    const entries = (
      await Promise.allSettled(names.map(async (name) => ({ name, info: await stat(join(this.dir, name)) })))
    ).flatMap((result) => (result.status === "fulfilled" ? [result.value] : []));
    const pinned = (name: string) => [...this.pinned].some((render) => render.has(name));
    let total = 0;
    for (const entry of entries.sort((a, b) => b.info.mtimeMs - a.info.mtimeMs)) {
      total += entry.info.size;
      if (total > this.maxBytes && !pinned(entry.name)) await unlink(join(this.dir, entry.name)).catch(() => {});
    }
  }
}
