// A chat channel's transcript since join, kept for the life of the channel
// store (the session), not of the panel: Tabs unmounts inactive pages and
// ChannelStore keeps only the newest frame. The App starts one per chat
// panel as soon as a manifest names it, so nothing received since join is
// lost before the page is first opened. Bounded to the newest MAX_LINES; a
// store reset (producer gone or changed) empties it.

import type { ChannelStore } from "@dimos/sdk";
import type { Manifest } from "@dimos/shared/manifest";

/** One chat.json.v1 line: ts is seconds since the epoch at encode time,
 * tool the tool name on the condensed call/result/progress lines. */
export interface ChatLine {
  role: string;
  text: string;
  ts: number;
  tool?: string;
}

export const MAX_LINES = 1000;

/** A frame's lines, with anything off-contract rendered as a visible line. */
function linesOf(value: unknown, frameTs: number): ChatLine[] {
  const items = Array.isArray(value) ? value : [null];
  return items.map((item): ChatLine => {
    const line = item as { role?: unknown; text?: unknown; ts?: unknown; tool?: unknown } | null;
    if (
      line === null || typeof line.role !== "string" || typeof line.text !== "string" ||
      typeof line.ts !== "number"
    ) {
      return { role: "unknown", text: "unreadable message", ts: frameTs };
    }
    return {
      role: line.role,
      text: line.text,
      ts: line.ts,
      tool: typeof line.tool === "string" ? line.tool : undefined,
    };
  });
}

export class ChatTranscript {
  #lines: ChatLine[] = [];
  #seen = 0;
  #listeners = new Set<() => void>();

  constructor(store: ChannelStore, readonly ch: string) {
    store.subscribe(ch, () => this.#pull(store));
    this.#pull(store);
  }

  #pull(store: ChannelStore): void {
    const slot = store.get(this.ch);
    if (slot === null) {
      if (this.#seen === 0) return;
      this.#seen = 0;
      this.#lines = [];
    } else if (slot.version > this.#seen) {
      this.#seen = slot.version;
      this.#lines = [...this.#lines, ...linesOf(slot.value, slot.ts)].slice(-MAX_LINES);
    } else {
      return;
    }
    for (const cb of this.#listeners) cb();
  }

  /** useSyncExternalStore pair: the array identity changes with the content. */
  subscribe = (cb: () => void): () => void => {
    this.#listeners.add(cb);
    return () => this.#listeners.delete(cb);
  };
  getSnapshot = (): ChatLine[] => this.#lines;
}

const transcripts = new WeakMap<ChannelStore, Map<string, ChatTranscript>>();

/** The transcript of `ch` on `store`, started on first use. */
export function chatTranscript(store: ChannelStore, ch: string): ChatTranscript {
  let byCh = transcripts.get(store);
  if (byCh === undefined) {
    byCh = new Map();
    transcripts.set(store, byCh);
  }
  let transcript = byCh.get(ch);
  if (transcript === undefined) {
    transcript = new ChatTranscript(store, ch);
    byCh.set(ch, transcript);
  }
  return transcript;
}

/** Start the transcript of every chat panel's message channel (slot 1). */
export function startChatTranscripts(store: ChannelStore, manifest: Manifest): void {
  for (const panel of manifest.panels) {
    if (panel.kind === "chat" && panel.channels.length === 4) {
      chatTranscript(store, panel.channels[1]);
    }
  }
}
