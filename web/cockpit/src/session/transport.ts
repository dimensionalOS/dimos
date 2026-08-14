// Reconnecting WebTransport wrapper: fetch /api/info, connect with the pinned
// cert hash, hand the session off, and retry forever with capped backoff when
// anything dies. /api/info is re-fetched on every attempt because a relay
// restart means a new QUIC port and a new ephemeral certificate.

import { PROTOCOL_VERSION } from "@dimos/shared";

export interface RelayInfo {
  wtUrl: string;
  certHash: string;
  v: number;
}

export type TransportPhase =
  | { phase: "connecting"; attempt: number }
  // connected: the session saw the relay's welcome (sessionReady()), not
  // just QUIC readiness - a reachable relay with a wedged session stays
  // "connecting".
  | { phase: "connected" }
  // reason: why the previous connection ended (e.g. the relay's kick reason
  // from WebTransportCloseInfo), when known.
  | { phase: "reconnecting"; attempt: number; retryAtMs: number; reason?: string }
  | { phase: "failed"; reason: string };

// Structural subset of WebTransport so tests (and later non-browser hosts) can
// fake it. The real WebTransport satisfies this as-is.
export interface BidiStreamLike {
  readable: ReadableStream<Uint8Array>;
  writable: WritableStream<Uint8Array>;
}

export interface WebTransportLike {
  ready: Promise<unknown>;
  closed: Promise<unknown>;
  close(): void;
  createBidirectionalStream(): Promise<BidiStreamLike>;
  incomingUnidirectionalStreams: ReadableStream<ReadableStream<Uint8Array>>;
}

export interface TransportEvents {
  onPhase(phase: TransportPhase): void;
  /** Run one session; resolving (or rejecting) means the session is over. */
  onSession(wt: WebTransportLike, info: RelayInfo): Promise<void>;
}

export interface TransportDeps {
  fetchInfo?: (signal: AbortSignal) => Promise<RelayInfo>;
  createWebTransport?: (info: RelayInfo) => WebTransportLike;
  now?: () => number;
}

export const BACKOFF_INITIAL_MS = 500;
export const BACKOFF_MAX_MS = 8000;
// Per-attempt deadline for /api/info + the QUIC handshake: a blackholed
// attempt must fail into backoff instead of pinning the phase at
// "connecting" forever.
export const CONNECT_TIMEOUT_MS = 10_000;

/** Delay before retry number `failures` (1-based): 500 ms doubling to 8 s. */
export function backoffDelayMs(failures: number): number {
  return Math.min(BACKOFF_INITIAL_MS * 2 ** (failures - 1), BACKOFF_MAX_MS);
}

/** `promise`, but rejecting as soon as `signal` aborts (timeout or stop()). */
function abortable<T>(promise: Promise<T>, signal: AbortSignal): Promise<T> {
  return new Promise<T>((resolve, reject) => {
    const onAbort = () => reject(new Error("aborted"));
    if (signal.aborted) {
      onAbort();
      return;
    }
    signal.addEventListener("abort", onAbort, { once: true });
    promise.then(resolve, reject);
  });
}

export async function fetchRelayInfo(signal: AbortSignal): Promise<RelayInfo> {
  const resp = await fetch("/api/info", { signal });
  if (!resp.ok) throw new Error(`/api/info returned ${resp.status}`);
  const data: unknown = await resp.json();
  if (
    typeof data !== "object" || data === null ||
    typeof (data as Record<string, unknown>).wtUrl !== "string" ||
    typeof (data as Record<string, unknown>).certHash !== "string" ||
    typeof (data as Record<string, unknown>).v !== "number"
  ) {
    throw new Error("/api/info returned an unexpected shape");
  }
  return data as unknown as RelayInfo;
}

function connectWebTransport(info: RelayInfo): WebTransportLike {
  const hash = Uint8Array.from(atob(info.certHash), (c) => c.charCodeAt(0));
  return new WebTransport(info.wtUrl, {
    serverCertificateHashes: [{ algorithm: "sha-256", value: hash }],
  });
}

export class ReconnectingTransport {
  #events: TransportEvents;
  #fetchInfo: (signal: AbortSignal) => Promise<RelayInfo>;
  #create: (info: RelayInfo) => WebTransportLike;
  #now: () => number;

  #phase: TransportPhase = { phase: "connecting", attempt: 1 };
  #wt: WebTransportLike | null = null;
  #timer: ReturnType<typeof setTimeout> | null = null;
  #wake: (() => void) | null = null;
  #abort: AbortController | null = null;
  #failures = 0;
  #started = false;
  #stopped = false;

  constructor(events: TransportEvents, deps: TransportDeps = {}) {
    this.#events = events;
    this.#fetchInfo = deps.fetchInfo ?? fetchRelayInfo;
    this.#create = deps.createWebTransport ?? connectWebTransport;
    this.#now = deps.now ?? Date.now;
  }

  get phase(): TransportPhase {
    return this.#phase;
  }

  start(): void {
    if (this.#started) return;
    this.#started = true;
    void this.#loop();
  }

  /** Stop retrying and close the current connection. No further events. */
  stop(): void {
    if (this.#stopped) return;
    this.#stopped = true;
    if (this.#timer !== null) clearTimeout(this.#timer);
    this.#abort?.abort();
    this.#wake?.();
    this.#closeCurrent();
  }

  /**
   * Session-level success (the relay's welcome arrived): publish the
   * connected phase and reset the backoff. A connection that dies before
   * welcome keeps counting as a failure.
   */
  sessionReady(): void {
    if (this.#stopped) return;
    this.#failures = 0;
    if (this.#phase.phase !== "connected") this.#setPhase({ phase: "connected" });
  }

  /** Terminal failure (protocol mismatch, no WebTransport support, ...). */
  fail(reason: string): void {
    if (this.#stopped) return;
    this.#setPhase({ phase: "failed", reason });
    this.stop();
  }

  #setPhase(phase: TransportPhase): void {
    this.#phase = phase;
    this.#events.onPhase(phase);
  }

  #closeCurrent(): void {
    try {
      this.#wt?.close();
    } catch {
      // already closed
    }
    this.#wt = null;
  }

  async #loop(): Promise<void> {
    let endReason: string | null = null;
    while (!this.#stopped) {
      this.#setPhase({ phase: "connecting", attempt: this.#failures + 1 });
      const abort = new AbortController();
      this.#abort = abort;
      const deadline = setTimeout(() => abort.abort(), CONNECT_TIMEOUT_MS);
      try {
        const info = await this.#fetchInfo(abort.signal);
        if (this.#stopped) return;
        if (info.v !== PROTOCOL_VERSION) {
          this.fail(`relay speaks protocol v${info.v}, this page speaks v${PROTOCOL_VERSION}`);
          return;
        }
        const wt = this.#create(info);
        this.#wt = wt;
        await abortable(wt.ready, abort.signal);
        if (this.#stopped) return;
        clearTimeout(deadline);
        // The session run and the connection death both end the session; wait
        // for whichever comes first, then drop the connection. The connected
        // phase is published by sessionReady() once the session saw welcome.
        await Promise.race([
          this.#events.onSession(wt, info).catch(() => {}),
          wt.closed.then(
            (info) => {
              const reason = (info as { reason?: string } | undefined)?.reason;
              if (reason) endReason = reason;
            },
            (e) => {
              endReason = String(e);
            },
          ),
        ]);
      } catch {
        // fetch or handshake failed or timed out; fall through to backoff
      } finally {
        clearTimeout(deadline);
        this.#abort = null;
      }
      this.#closeCurrent();
      if (this.#stopped) return;
      this.#failures += 1;
      const delay = backoffDelayMs(this.#failures);
      this.#setPhase({
        phase: "reconnecting",
        attempt: this.#failures,
        retryAtMs: this.#now() + delay,
        ...(endReason !== null ? { reason: endReason } : {}),
      });
      endReason = null;
      await this.#sleep(delay);
    }
  }

  #sleep(ms: number): Promise<void> {
    return new Promise((resolve) => {
      this.#wake = resolve;
      this.#timer = setTimeout(() => {
        this.#timer = null;
        this.#wake = null;
        resolve();
      }, ms);
    });
  }
}
