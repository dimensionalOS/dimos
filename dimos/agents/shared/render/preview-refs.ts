import { mkdtemp, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { extname, join } from "node:path";
import type { ImageContent, Message } from "@earendil-works/pi-ai";
import type { FileRef, RenderDetails, RenderedView } from "#dimos/agents/shared/protocol/media.ts";
import { RenderDetailsSchema } from "#dimos/agents/shared/protocol/media.ts";
import { clipRenderer } from "#dimos/agents/shared/render/clip.ts";
import { readVerified } from "#dimos/agents/shared/render/files.ts";
import { RenderCache, readSource } from "#dimos/agents/shared/render/lib/artifacts.ts";
import { modelImage } from "#dimos/agents/shared/render/lib/model-image.ts";
import { fitCamera, paintCloud, parseCloud } from "#dimos/agents/shared/render/points.ts";
import { Type } from "typebox";
import { Value } from "typebox/value";

const REBUILD_MAX_BYTES = 64 * 2 ** 20;

/** Caption marker for humans / TUI; never used as authority to read files. */
export const PREVIEW_PREFIX = "Preview: ";

const RenderedSchema = Type.Object({ render: RenderDetailsSchema });

/** The on-disk terminal preview (sheet for a clip); captions only. */
export function previewRefOf(view: RenderedView): FileRef {
  return view.kind === "clip" ? view.sheet : view.preview;
}

export function previewPathsFromDetails(details: RenderDetails): string[] {
  return [...new Set(details.views.map((view) => previewRefOf(view).path))];
}

/** Structured `{ render }` tool details only — never free-form Preview: text (path injection). */
export function renderDetailsOf(details: unknown): RenderDetails | null {
  if (Value.Check(RenderedSchema, details) && details.render.views.length > 0) return details.render;
  return null;
}

/**
 * Re-render a clip contact sheet when the cache entry was pruned.
 * Seals the exact bytes verified against `view.clip.frames` into a temp index (with those hashes),
 * then renders that sealed copy — the renderer never re-reads the live source tree.
 */
async function rebuildClipSheet(view: Extract<RenderedView, { kind: "clip" }>): Promise<ImageContent> {
  const indexBytes = await readVerified(view.source);
  const indexJson: unknown = JSON.parse(indexBytes.toString("utf8"));
  if (
    typeof indexJson !== "object" ||
    indexJson === null ||
    !("type" in indexJson) ||
    (indexJson.type !== "image" && indexJson.type !== "points")
  ) {
    throw new Error("clip index is not a frame index");
  }
  const timeOrigin =
    "timeOrigin" in indexJson && typeof indexJson.timeOrigin === "number" ? indexJson.timeOrigin : undefined;

  const dir = await mkdtemp(join(tmpdir(), "clip-hydrate-"));
  try {
    const sealedFrames: { path: string; timestamp: number; sha256: string }[] = [];
    for (const frame of view.clip.frames) {
      const bytes = await readVerified(frame.source);
      const ext = extname(frame.source.path) || (indexJson.type === "points" ? ".json" : ".bin");
      const name = `frame-${frame.index}${ext}`;
      await writeFile(join(dir, name), bytes);
      sealedFrames.push({ path: name, timestamp: frame.timestamp, sha256: frame.source.sha256 });
    }
    const sealedIndexPath = join(dir, "index.json");
    await writeFile(
      sealedIndexPath,
      JSON.stringify({
        type: indexJson.type,
        ...(timeOrigin === undefined ? {} : { timeOrigin }),
        frames: sealedFrames,
      }),
    );
    const cache = new RenderCache(join(dir, "cache"), REBUILD_MAX_BYTES);
    const source = await readSource(sealedIndexPath, REBUILD_MAX_BYTES, null);
    const part = await cache.pinning((previewCache) =>
      clipRenderer.render(source, view.label, {
        cache: previewCache,
        maxSourceBytes: REBUILD_MAX_BYTES,
        onProgress: null,
      }),
    );
    return part.image.content;
  } finally {
    await rm(dir, { recursive: true, force: true });
  }
}

/**
 * Rebuild the model image from verified FileRefs in render details.
 * Images come from the source (correct JPEG/PNG policy); points/clips use the cached raster when present,
 * and fall back to re-rendering the verified source when the cache was pruned.
 */
export async function imageContentFromView(view: RenderedView): Promise<ImageContent> {
  if (view.kind === "image") {
    return (await modelImage(await readVerified(view.source))).content;
  }
  if (view.kind === "points") {
    try {
      return (await modelImage(await readVerified(view.preview), "graphic")).content;
    } catch {
      const bytes = await readVerified(view.source);
      const cloud = parseCloud(JSON.parse(bytes.toString("utf8")));
      const png = await paintCloud(cloud, fitCamera(cloud), { width: 900, height: 448 });
      return (await modelImage(png, "graphic")).content;
    }
  }
  try {
    return (await modelImage(await readVerified(view.sheet))).content;
  } catch {
    return rebuildClipSheet(view);
  }
}

/**
 * Inject images into toolResult messages for the upcoming LLM call from trusted render details only.
 * Session history keeps path-only content; this clone is request-scoped.
 */
export async function hydrateToolResultPreviews(messages: readonly Message[]): Promise<Message[]> {
  let changed = false;
  const next: Message[] = [];
  for (const message of messages) {
    if (message.role !== "toolResult") {
      next.push(message);
      continue;
    }
    if (typeof message.content === "string") {
      next.push(message);
      continue;
    }
    const content = message.content;
    if (content.some((block) => block.type === "image")) {
      next.push(message);
      continue;
    }
    const render = renderDetailsOf(message.details);
    if (render === null) {
      next.push(message);
      continue;
    }
    const images: ImageContent[] = [];
    for (const view of render.views) {
      try {
        images.push(await imageContentFromView(view));
      } catch {
        // Drop views whose verified sources are gone; text still names paths.
      }
    }
    if (!images.length) {
      next.push(message);
      continue;
    }
    changed = true;
    next.push({ ...message, content: [...content, ...images] });
  }
  return changed ? next : [...messages];
}
