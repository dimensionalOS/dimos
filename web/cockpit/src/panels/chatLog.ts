// Stateful side of the chat panel: one ChatLog per (store, channel) folds
// the chat channel's frames into rows through the pure reducer (chatFold.ts)
// and tracks the lines this viewer typed until the bridge echoes them back.
// Logs live in a module-level registry keyed by the store so the transcript
// survives the App's epoch remount (<main key={status.epoch}>): React only
// ever reads a snapshot. The log follows the store's own producer rule: a
// store.reset() (the SDK invalidated the producer - robot left, manifest
// changed) empties the rows, since a restarted bridge restarts its entry
// numbers and the old rows would otherwise swallow the new run as
// duplicates; the bridge replays its last 200 entries on resubscribe.

import type { ChannelStore } from "@dimos/sdk";
import { foldChat, readChatMsg, type Row } from "./chatFold.ts";

/** A typed line without an echo for this long is presumed lost. */
export const RETRY_AFTER_MS = 5000;

export interface PendingLine {
  key: string;
  text: string;
  sentAt: number;
  status: "pending" | "failed";
}

export interface ChatLogSnapshot {
  rows: readonly Row[];
  pending: readonly PendingLine[];
}

export class ChatLog {
  #store: ChannelStore;
  #ch: string;
  #now: () => number;
  #rows: Row[] = [];
  #pending: PendingLine[] = [];
  #snapshot: ChatLogSnapshot = { rows: [], pending: [] };
  #listeners = new Set<() => void>();
  #timer: ReturnType<typeof setTimeout> | null = null;
  #foldedVersion = -1;
  #nextKey = 1;

  constructor(store: ChannelStore, ch: string, now: () => number = Date.now) {
    this.#store = store;
    this.#ch = ch;
    this.#now = now;
    // Never unsubscribed: the log lives as long as the store it observes.
    store.subscribe(ch, this.#onIngest);
    this.#onIngest(); // a slot may predate the log
  }

  subscribe = (cb: () => void): () => void => {
    this.#listeners.add(cb);
    return () => this.#listeners.delete(cb);
  };

  getSnapshot = (): ChatLogSnapshot => this.#snapshot;

  /** Record a line that was accepted for sending; it shows as pending until
   * the bridge echoes a human message with the same text. */
  addPending(text: string): PendingLine {
    const line: PendingLine = {
      key: `pending-${this.#nextKey++}`,
      text: text.trim(),
      sentAt: this.#now(),
      status: "pending",
    };
    this.#pending = [...this.#pending, line];
    this.#armTimer();
    this.#emit();
    return line;
  }

  /** The line was sent again: back to pending with a fresh deadline. */
  resent(key: string): void {
    const line = this.#pending.find((p) => p.key === key);
    if (line === undefined) return;
    this.#pending = this.#pending.map((p) =>
      p === line ? { ...p, sentAt: this.#now(), status: "pending" } : p
    );
    this.#armTimer();
    this.#emit();
  }

  dismiss(key: string): void {
    const next = this.#pending.filter((p) => p.key !== key);
    if (next.length === this.#pending.length) return;
    this.#pending = next;
    this.#emit();
  }

  #onIngest = (): void => {
    const slot = this.#store.get(this.#ch);
    if (slot === null) {
      // store.reset(): producer invalidated (see the header comment).
      this.#foldedVersion = -1;
      if (this.#rows.length > 0) {
        this.#rows = [];
        this.#emit();
      }
      return;
    }
    if (slot.version === this.#foldedVersion) return;
    this.#foldedVersion = slot.version;
    const msg = readChatMsg(slot.value, slot.ts);
    if (msg === null) return;
    const rows = foldChat(this.#rows, msg);
    let changed = rows !== this.#rows;
    this.#rows = rows;
    if (msg.role === "human" && this.#settle(msg.content)) changed = true;
    if (changed) this.#emit();
  };

  /** An echoed human line settles the oldest pending line with that text. */
  #settle(content: string): boolean {
    const text = content.trim();
    const at = this.#pending.findIndex((p) => p.text === text);
    if (at === -1) return false;
    this.#pending = this.#pending.filter((_, i) => i !== at);
    return true;
  }

  #armTimer(): void {
    if (this.#timer !== null) clearTimeout(this.#timer);
    const oldest = this.#pending.find((p) => p.status === "pending");
    if (oldest === undefined) {
      this.#timer = null;
      return;
    }
    const delay = Math.max(0, oldest.sentAt + RETRY_AFTER_MS - this.#now());
    this.#timer = setTimeout(this.#expire, delay);
  }

  #expire = (): void => {
    this.#timer = null;
    const now = this.#now();
    let changed = false;
    this.#pending = this.#pending.map((p) => {
      if (p.status === "pending" && now - p.sentAt >= RETRY_AFTER_MS) {
        changed = true;
        return { ...p, status: "failed" };
      }
      return p;
    });
    this.#armTimer();
    if (changed) this.#emit();
  };

  #emit(): void {
    this.#snapshot = { rows: this.#rows, pending: this.#pending };
    for (const cb of this.#listeners) {
      try {
        cb();
      } catch (e) {
        console.error("chat log subscriber threw", e);
      }
    }
  }
}

const logsByStore = new WeakMap<ChannelStore, Map<string, ChatLog>>();

/** The one ChatLog for this store + channel (created on first use). */
export function chatLogFor(store: ChannelStore, ch: string): ChatLog {
  let logs = logsByStore.get(store);
  if (logs === undefined) {
    logs = new Map();
    logsByStore.set(store, logs);
  }
  let log = logs.get(ch);
  if (log === undefined) {
    log = new ChatLog(store, ch);
    logs.set(ch, log);
  }
  return log;
}
