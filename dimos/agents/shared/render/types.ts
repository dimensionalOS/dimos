import type { FileRef, RenderDetails } from "#dimos/agents/shared/protocol/media.ts";
import type { ImageContent } from "@earendil-works/pi-ai";

export type RenderProgress = {
  readonly done: number;
  readonly total: number;
  readonly latest: FileRef | null;
};

/** A rendering as it enters a tool result or an input: caption lines, one image per view, and the details. */
export type Rendering = {
  /** One line per view (label, source path, SHA-256), one notice per listed file, one line per failed file. */
  readonly text: string;
  readonly images: readonly ImageContent[];
  readonly details: RenderDetails;
};
