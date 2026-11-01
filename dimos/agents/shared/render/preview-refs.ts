import { readFile } from "node:fs/promises";
import type { ImageContent, Message } from "@earendil-works/pi-ai";
import type { FileRef, RenderDetails, RenderedView } from "#dimos/agents/shared/protocol/media.ts";
import { RenderDetailsSchema } from "#dimos/agents/shared/protocol/media.ts";
import { modelImage } from "#dimos/agents/shared/render/lib/model-image.ts";
import { Type } from "typebox";
import { Value } from "typebox/value";

/** Stable marker in render caption text; hydrate falls back to these paths when details are absent. */
export const PREVIEW_PREFIX = "Preview: ";

const RenderedSchema = Type.Object({ render: RenderDetailsSchema });

/** The on-disk preview the model should see for one view (sheet for a clip). */
export function previewRefOf(view: RenderedView): FileRef {
  return view.kind === "clip" ? view.sheet : view.preview;
}

export function previewPathsFromDetails(details: RenderDetails): string[] {
  return [...new Set(details.views.map((view) => previewRefOf(view).path))];
}

export function previewPathsFromText(text: string): string[] {
  const paths: string[] = [];
  for (const line of text.split(/\r?\n/)) {
    if (!line.startsWith(PREVIEW_PREFIX)) continue;
    const path = line.slice(PREVIEW_PREFIX.length).trim();
    if (path) paths.push(path);
  }
  return [...new Set(paths)];
}

/** Prefer structured `{ render }` tool details; fall back to Preview: lines in text. */
export function previewPathsFromToolResult(details: unknown, text: string): string[] {
  if (Value.Check(RenderedSchema, details) && details.render.views.length > 0)
    return previewPathsFromDetails(details.render);
  return previewPathsFromText(text);
}

/** Load a cached preview for the LLM call only — never store this on the tool result. */
export async function imageContentFromPreview(path: string): Promise<ImageContent> {
  const bytes = await readFile(path);
  return (await modelImage(bytes)).content;
}

/**
 * Inject preview images into toolResult (and path-only user) messages for the upcoming LLM call.
 * Session history / gateway events keep path-only content; this clone is request-scoped.
 */
export async function hydrateToolResultPreviews(messages: readonly Message[]): Promise<Message[]> {
  let changed = false;
  const next: Message[] = [];
  for (const message of messages) {
    if (message.role !== "toolResult" && message.role !== "user") {
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
    const text = content
      .flatMap((block) => (block.type === "text" && typeof block.text === "string" ? [block.text] : []))
      .join("\n");
    const paths =
      message.role === "toolResult"
        ? previewPathsFromToolResult(message.details, text)
        : previewPathsFromText(text);
    if (!paths.length) {
      next.push(message);
      continue;
    }
    const images: ImageContent[] = [];
    for (const path of paths) {
      try {
        images.push(await imageContentFromPreview(path));
      } catch {
        // Drop missing previews; text still names the path for the model.
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
