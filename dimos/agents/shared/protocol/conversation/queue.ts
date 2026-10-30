import type { DimosInput } from "./input.ts";
import type { InboxItem } from "@earendil-works/pi-durable";

/** What `withdraw` and `abort` hand back: the user's queued text, in queue order, for the editor. */
export type QueuedText = { readonly mode: "steer" | "followUp"; readonly text: string };

/**
 * The `dimos.input` data of a queued item, or null for a write or an item the harness did not place (every input goes
 * through InputGate, so one is never seen). The inbox document is JSON; this is where its data takes its one type.
 */
export function inputOf(item: InboxItem): DimosInput | null {
  if (item.mode === "write") return null;
  const data: unknown = item.entry?.data;
  if (typeof data !== "object" || data === null || !("source" in data) || !("text" in data)) return null;
  return data as DimosInput;
}
