import {
  encodeDataFrame,
  MAX_TOPIC_GUARD_BYTES,
  MAX_TOPIC_GUARDS,
  type Msg,
  TOPIC_GUARD_CHANNEL,
  topicGuardMeta,
  type TopicMsg,
  type TopicSourceInfo,
} from "@dimos/shared";
import { TopicError, type TopicPublishReceipt, type TopicStatus } from "./topics.ts";

export interface TopicPublisherOptions {
  source?: string;
  encoding?: string;
  /** Handle expiry, 100–1000 ms without an accepted command, even with heartbeats. */
  timeoutMs?: number;
  /** Ownership group; distinct keys in the same controller share ownership. */
  group: string;
  /** Explicit takeover stops the displaced owner before this handle is ready. */
  takeover?: boolean;
}
export interface TopicPublisher {
  ready(): Promise<void>;
  status(): TopicStatus;
  onStatus(callback: (status: TopicStatus) => void): () => void;
  /** Bounded, expiring command. No enqueue while opening and no reconnect retry. */
  publish(payload: Uint8Array): Promise<TopicPublishReceipt>;
  /** Idempotent safe stop and terminal release. A fresh input needs a new handle. */
  close(): void;
}
interface Host {
  id(): string;
  source(source?: string): TopicSourceInfo | undefined;
  send(msg: Msg): void;
  binary(bytes: Uint8Array): Promise<void>;
}
type Pending = {
  resolve(receipt: TopicPublishReceipt): void;
  reject(error: TopicError): void;
};

class Publisher implements TopicPublisher {
  readonly id: string;
  readonly source: string;
  readonly gen: number;
  readonly timeoutMs: number;
  handle: string | null = null;
  ticket = 0;
  #status: TopicStatus = { phase: "connecting" };
  #listeners = new Set<(status: TopicStatus) => void>();
  #pending = new Map<number, Pending>();
  #sequence = 0;
  #lastBeat = performance.now();
  #lastTicket = performance.now();
  #timer: ReturnType<typeof setInterval>;
  #opening: ReturnType<typeof setTimeout>;
  #resolve!: () => void;
  #reject!: (error: TopicError) => void;
  #ready = new Promise<void>((resolve, reject) => {
    this.#resolve = resolve;
    this.#reject = reject;
  });

  constructor(
    readonly owner: TopicPublishers,
    readonly host: Host,
    key: string,
    stop: Uint8Array,
    options: TopicPublisherOptions,
    info: TopicSourceInfo,
  ) {
    this.id = host.id();
    this.source = info.source;
    this.gen = info.gen;
    this.timeoutMs = options.timeoutMs ?? 400;
    this.#ready.catch(() => {});
    this.#timer = setInterval(() => {
      if (this.#status.phase !== "ready") return;
      const now = performance.now();
      if (now - this.#lastBeat >= this.timeoutMs) {
        this.fail("browser_suspended");
        return;
      }
      if (now - this.#lastTicket >= this.timeoutMs) {
        this.fail("guard_heartbeat_timeout");
        return;
      }
      this.#lastBeat = now;
      host.send({
        t: "topic_guard_heartbeat",
        handle: this.handle!,
        gen: this.gen,
        ticket: this.ticket,
      });
    }, this.timeoutMs / 4);
    this.#opening = setTimeout(() => this.fail("guard_ready_timeout"), 15_000);
    const meta = topicGuardMeta({
      action: "open",
      id: this.id,
      source: this.source,
      gen: this.gen,
      key,
      encoding: options.encoding ?? "application/octet-stream",
      group: options.group,
      timeoutMs: this.timeoutMs,
      takeover: options.takeover ?? false,
    });
    if (!meta || !stop.length || stop.length > MAX_TOPIC_GUARD_BYTES) {
      queueMicrotask(() => this.fail("invalid_guard_options", "rejected"));
      return;
    }
    // Encoding copies the caller's safe stop; later buffer mutation cannot
    // change what the provider will publish on expiry.
    host.binary(encodeDataFrame({
      ch: TOPIC_GUARD_CHANNEL,
      seq: 0,
      ts: Date.now() / 1000,
      delivery: "reliable",
      meta,
    }, stop)).catch(() => this.fail("guard_transport_failed"));
  }

  ready(): Promise<void> {
    return this.#ready;
  }
  status(): TopicStatus {
    return this.#status;
  }
  onStatus(callback: (status: TopicStatus) => void): () => void {
    this.#listeners.add(callback);
    callback(this.#status);
    return () => this.#listeners.delete(callback);
  }
  #transition(status: TopicStatus): void {
    this.#status = status;
    for (const callback of [...this.#listeners]) {
      try {
        callback(status);
      } catch (error) {
        console.error("publisher status callback failed", error);
      }
    }
  }
  opened(handle: string, ticket: number, timeoutMs: number): void {
    this.handle = handle;
    if (this.#status.phase !== "connecting" || timeoutMs !== this.timeoutMs) {
      this.close();
      return;
    }
    clearTimeout(this.#opening);
    this.ticket = ticket;
    this.#lastBeat = performance.now();
    this.#lastTicket = this.#lastBeat;
    this.#transition({ phase: "ready" });
    this.#resolve();
  }
  publish(payload: Uint8Array): Promise<TopicPublishReceipt> {
    if (performance.now() - this.#lastBeat >= this.timeoutMs) this.fail("browser_suspended");
    if (performance.now() - this.#lastTicket >= this.timeoutMs) {
      this.fail("guard_heartbeat_timeout");
    }
    if (this.#status.phase !== "ready") {
      return Promise.reject(new TopicError("guard_not_ready", "rejected"));
    }
    if (payload.length > MAX_TOPIC_GUARD_BYTES || this.#pending.size >= 8) {
      return Promise.reject(new TopicError("guard_busy", "rejected"));
    }
    const seq = ++this.#sequence;
    const result = new Promise<TopicPublishReceipt>((resolve, reject) => {
      this.#pending.set(seq, { resolve, reject });
    });
    this.host.binary(encodeDataFrame({
      ch: TOPIC_GUARD_CHANNEL,
      seq,
      ts: Date.now() / 1000,
      delivery: "reliable",
      meta: {
        action: "command",
        source: this.source,
        gen: this.gen,
        handle: this.handle!,
        ticket: this.ticket,
      },
    }, payload)).catch(() => this.fail("guard_transport_failed"));
    return result;
  }
  receipt(
    seq: number,
    outcome: "accepted" | "rejected" | "unknown",
    code: string,
    providerTs: number,
  ): void {
    const pending = this.#pending.get(seq);
    if (!pending) return;
    this.#pending.delete(seq);
    if (outcome === "accepted") pending.resolve({ id: `${this.handle}:${seq}`, providerTs });
    else pending.reject(new TopicError(code, outcome));
  }
  refresh(ticket: number): void {
    if (ticket > this.ticket) {
      this.ticket = ticket;
      this.#lastTicket = performance.now();
    }
  }
  fail(code: string, phase: "rejected" | "interrupted" | "closed" = "interrupted"): void {
    clearInterval(this.#timer);
    clearTimeout(this.#opening);
    // A stop remains meaningful even when a late ready arrives after close.
    if (this.handle !== null) {
      this.host.send({ t: "topic_guard_stop", handle: this.handle, gen: this.gen });
    }
    const error = new TopicError(code, phase === "rejected" ? "rejected" : "interrupted");
    this.#reject(error);
    for (const pending of this.#pending.values()) pending.reject(new TopicError(code, "unknown"));
    this.#pending.clear();
    if (!["closed", "rejected", "interrupted"].includes(this.#status.phase)) {
      this.#transition({ phase, error });
    }
    this.owner.release(this);
  }
  close(): void {
    this.fail("released", "closed");
  }
}

export class TopicPublishers {
  #publishers = new Map<string, Publisher>();
  #handles = new Map<string, Publisher>();
  #focusStop = () => this.stopAll("focus_lost");
  #visibility = () => {
    if (document.hidden) this.#focusStop();
  };
  constructor(readonly host: Host) {
    if (typeof window !== "undefined") {
      globalThis.addEventListener("blur", this.#focusStop);
      globalThis.addEventListener("pagehide", this.#focusStop);
      document.addEventListener("visibilitychange", this.#visibility);
    }
  }
  publisher(key: string, stop: Uint8Array, options: TopicPublisherOptions): TopicPublisher {
    if (
      !(stop instanceof Uint8Array) ||
      Object.keys(options).some((key) =>
        !["source", "encoding", "timeoutMs", "group", "takeover"].includes(key)
      )
    ) {
      throw new TopicError("invalid_guard_options", "rejected");
    }
    const info = this.host.source(options.source);
    if (!info?.write || !info.guardedPublishers) {
      throw new TopicError("guarded_publishers_unavailable", "rejected");
    }
    // The visibility listener stops armed handles; a handle opened from a
    // hidden page would arm after that stop and drive the robot unseen.
    if (typeof document !== "undefined" && document.hidden) {
      throw new TopicError("focus_lost", "rejected");
    }
    if (this.#publishers.size >= MAX_TOPIC_GUARDS) throw new TopicError("guard_limit", "rejected");
    const publisher = new Publisher(this, this.host, key, stop, options, info);
    this.#publishers.set(publisher.id, publisher);
    return publisher;
  }
  release(publisher: Publisher): void {
    this.#publishers.delete(publisher.id);
    if (publisher.handle !== null) this.#handles.delete(publisher.handle);
  }
  message(msg: TopicMsg): boolean {
    if (msg.t === "topic_guard_ready") {
      const publisher = this.#publishers.get(msg.id);
      if (!publisher || publisher.gen !== msg.gen) {
        this.host.send({ t: "topic_guard_stop", handle: msg.handle, gen: msg.gen });
      } else {
        this.#handles.set(msg.handle, publisher);
        publisher.opened(msg.handle, msg.ticket, msg.timeoutMs);
      }
      return true;
    }
    if (msg.t === "topic_error") {
      const publisher = this.#publishers.get(msg.id);
      if (!publisher) return false;
      publisher.fail(msg.code, "rejected");
      return true;
    }
    if (
      msg.t === "topic_guard_closed" || msg.t === "topic_guard_ticket" ||
      msg.t === "topic_guard_result"
    ) {
      const publisher = this.#handles.get(msg.handle);
      if (!publisher || publisher.gen !== msg.gen) return true;
      if (msg.t === "topic_guard_closed") publisher.fail(msg.code);
      else if (msg.t === "topic_guard_ticket") publisher.refresh(msg.ticket);
      else publisher.receipt(msg.seq, msg.outcome, msg.code, msg.providerTs);
      return true;
    }
    return false;
  }
  sourcesChanged(): void {
    for (const publisher of [...this.#publishers.values()]) {
      if (this.host.source(publisher.source)?.gen !== publisher.gen) {
        publisher.fail("source_offline");
      }
    }
  }
  stopAll(code: string): void {
    for (const publisher of [...this.#publishers.values()]) publisher.fail(code);
  }
  close(): void {
    this.stopAll("closed");
    if (typeof window !== "undefined") {
      globalThis.removeEventListener("blur", this.#focusStop);
      globalThis.removeEventListener("pagehide", this.#focusStop);
      document.removeEventListener("visibilitychange", this.#visibility);
    }
  }
}
