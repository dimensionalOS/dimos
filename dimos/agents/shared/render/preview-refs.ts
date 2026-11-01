import type { ImageContent, Message } from "@earendil-works/pi-ai";
import type { FileRef, RenderDetails, RenderedView } from "#dimos/agents/shared/protocol/media.ts";
import { RenderDetailsSchema } from "#dimos/agents/shared/protocol/media.ts";
import { readVerified } from "#dimos/agents/shared/render/files.ts";
import { modelImage } from "#dimos/agents/shared/render/lib/model-image.ts";
import { fitCamera, paintCloud, parseCloud } from "#dimos/agents/shared/render/points.ts";
import { Type } from "typebox";
import { Value } from "typebox/value";

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
 * Rebuild the model image from verified FileRefs in render details.
 * Images come from the source (correct JPEG/PNG policy); points/clips use the cached raster when present,
 * and points fall back to re-painting the verified source JSON when the cache was pruned.
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
  return (await modelImage(await readVerified(view.sheet))).content;
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
