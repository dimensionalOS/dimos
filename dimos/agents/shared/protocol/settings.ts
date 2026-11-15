import type { ModelRef } from "@earendil-works/pi-durable";
import type { ModelThinkingLevel } from "@earendil-works/pi-ai";

export type Settings = {
  /** A new conversation's directory; null means the DimOS checkout. */
  readonly workspace: string | null;
  readonly model: ModelRef | null;
  readonly thinkingLevel: ModelThinkingLevel | null;
  readonly theme: string | null;
};
