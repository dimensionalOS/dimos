import type { FileRef, RenderDetails } from "#dimos/agents/shared/protocol/media.ts";
import type { ImageContent } from "@earendil-works/pi-ai";

export type RenderProgress = {
  readonly done: number;
  readonly total: number;
  readonly latest: FileRef | null;
};

/**
 * A rendering: caption lines (including Preview: paths), optional eager images for local callers, and details.
 * Stored tool results / inputs must keep path-only content; `beforeRequest` hydrates images for the LLM.
 */
export type Rendering = {
  /** One caption (with Preview: path) per view, one notice per listed file, one line per failed file. */
  readonly text: string;
  /** Eager model images for /inspect and tests; do not spread into stored tool results. */
  readonly images: readonly ImageContent[];
  readonly details: RenderDetails;
};
