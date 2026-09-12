import { createHash } from "node:crypto";
import {
  mkdir,
  readFile,
  readdir,
  stat,
  unlink,
  writeFile,
} from "node:fs/promises";
import { join } from "node:path";
import type { Paths } from "./config.js";

export async function readArtifact(
  path: string,
  digest?: string,
): Promise<{ bytes: Buffer; sha256: string }> {
  if ((await stat(path)).size > 32 * 1024 * 1024)
    throw new Error("Export exceeds 32 MiB; select a smaller view in DimOS.");
  const bytes = await readFile(path);
  const sha256 = createHash("sha256").update(bytes).digest("hex");
  if (digest && digest !== sha256)
    throw new Error(
      "Source changed since the agent inspected it; retained preview remains available.",
    );
  return { bytes, sha256 };
}

export async function cacheImage(
  paths: Paths,
  png: Buffer,
  extension: "png" | "gif" = "png",
): Promise<string> {
  await mkdir(paths.cache, { recursive: true, mode: 0o700 });
  const path = join(
    paths.cache,
    createHash("sha256").update(png).digest("hex") + "." + extension,
  );
  await writeFile(path, png, { mode: 0o600 });
  const entries = (
    await Promise.allSettled(
      (await readdir(paths.cache))
        .filter((name) => /^[a-f0-9]{64}\.(png|gif)$/.test(name))
        .map(async (name) => ({
          name,
          info: await stat(join(paths.cache, name)),
        })),
    )
  ).flatMap((result) => (result.status === "fulfilled" ? [result.value] : []));
  let bytes = 0;
  for (const entry of entries.sort((a, b) => b.info.mtimeMs - a.info.mtimeMs)) {
    bytes += entry.info.size;
    if (bytes > 128 * 1024 * 1024 && entry.name !== path.split("/").pop())
      await unlink(join(paths.cache, entry.name)).catch(() => {});
  }
  return path;
}
