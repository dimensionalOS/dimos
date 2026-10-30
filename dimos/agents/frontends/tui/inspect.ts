import type { FileRenderer } from "../../shared/render/render.ts";
import type { ProgressPanel } from "./conversation/progress.ts";
import type { RenderDetails } from "../../shared/protocol/media.ts";
import { basename, resolve } from "node:path";

/**
 * `/inspect PATH`: renders the file in this process, as the gateway would for a tool result, reporting clip progress
 * to `progress`. Throws when the file is not renderable; the original file is only read.
 */
export async function inspectPath(
  path: string,
  cwd: string,
  renderer: FileRenderer,
  progress: ProgressPanel,
): Promise<RenderDetails> {
  const absolute = resolve(cwd, path);
  const rendering = await renderer.render([{ path: absolute, sha256: null }], basename(absolute), (update) =>
    progress.update(update),
  );
  if (rendering === null) throw new Error(`${absolute} is not an image, a point cloud or a frame index`);
  return rendering.details;
}
