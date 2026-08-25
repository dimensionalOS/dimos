import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { PROTOCOL_VERSION } from "@dimos/shared";
import {
  backoffDelayMs,
  CONNECT_TIMEOUT_MS,
  ReconnectingTransport,
  type RelayInfo,
  type TransportPhase,
  type WebTransportLike,
} from "./transport.ts";

const INFO: RelayInfo = {
  wtUrl: "https://127.0.0.1:1234/viewer",
  certHash: "aGFzaA==",
  v: PROTOCOL_VERSION,
};

interface FakeWt extends WebTransportLike {
  resolveClosed(info?: unknown): void;
  rejectClosed(): void;
}

function makeFakeWt({ readyFails = false, readyHangs = false } = {}): FakeWt {
  let resolveClosed = (_info?: unknown) => {};
  let rejectClosed = () => {};
  const closed = new Promise<unknown>((resolve, reject) => {
    resolveClosed = (info?: unknown) => resolve(info);
    rejectClosed = () => reject(new Error("connection lost"));
  });
  let ready: Promise<unknown> = Promise.resolve();
  if (readyFails) ready = Promise.reject(new Error("handshake failed"));
  if (readyHangs) ready = new Promise(() => {});
  return {
    ready,
    closed,
    close: vi.fn(),
    createBidirectionalStream: () => Promise.reject(new Error("not used in transport tests")),
    incomingUnidirectionalStreams: new ReadableStream(),
    datagrams: { writable: new WritableStream() },
    resolveClosed,
    rejectClosed,
  };
}

describe("backoffDelayMs", () => {
  it("doubles from 500 ms and caps at 8 s", () => {
    expect([1, 2, 3, 4, 5, 6, 7].map(backoffDelayMs)).toEqual([
      500,
      1000,
      2000,
      4000,
      8000,
      8000,
      8000,
    ]);
  });
});

describe("ReconnectingTransport", () => {
  let phases: TransportPhase[];

  beforeEach(() => {
    vi.useFakeTimers();
    phases = [];
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  function makeTransport(opts: {
    fetchInfo: (signal: AbortSignal) => Promise<RelayInfo>;
    createWebTransport?: (info: RelayInfo) => WebTransportLike;
    onSession?: (wt: WebTransportLike) => Promise<void>;
    // welcome normally arrives right after connect; false keeps the session
    // from signalling readiness so the pre-welcome phase can be observed.
    autoReady?: boolean;
  }) {
    const transport: ReconnectingTransport = new ReconnectingTransport(
      {
        onPhase: (p) => phases.push(p),
        onSession: (wt) => {
          if (opts.autoReady !== false) transport.sessionReady();
          return opts.onSession !== undefined ? opts.onSession(wt) : new Promise(() => {});
        },
      },
      {
        fetchInfo: opts.fetchInfo,
        createWebTransport: opts.createWebTransport ?? (() => makeFakeWt()),
      },
    );
    return transport;
  }

  it("retries with doubling backoff and re-fetches /api/info every attempt", async () => {
    const fetchInfo = vi.fn(() => Promise.reject(new Error("relay down")));
    const transport = makeTransport({ fetchInfo });
    transport.start();

    await vi.advanceTimersByTimeAsync(0);
    expect(fetchInfo).toHaveBeenCalledTimes(1);
    expect(phases).toEqual([
      { phase: "connecting", attempt: 1 },
      { phase: "reconnecting", attempt: 1, retryAtMs: Date.now() + 500 },
    ]);

    await vi.advanceTimersByTimeAsync(499);
    expect(fetchInfo).toHaveBeenCalledTimes(1);
    await vi.advanceTimersByTimeAsync(1);
    expect(fetchInfo).toHaveBeenCalledTimes(2);

    await vi.advanceTimersByTimeAsync(1000);
    expect(fetchInfo).toHaveBeenCalledTimes(3);
    const delays = phases
      .filter((p) => p.phase === "reconnecting")
      .map((p) => (p.phase === "reconnecting" ? p.attempt : 0));
    expect(delays).toEqual([1, 2, 3]);
    transport.stop();
  });

  it("treats a dying connection as a failed attempt and resets after success", async () => {
    const wts: FakeWt[] = [];
    const fetchInfo = vi.fn(() => Promise.resolve(INFO));
    const transport = makeTransport({
      fetchInfo,
      createWebTransport: () => {
        const wt = makeFakeWt();
        wts.push(wt);
        return wt;
      },
    });
    transport.start();

    await vi.advanceTimersByTimeAsync(0);
    expect(transport.phase).toEqual({ phase: "connected" });
    expect(wts).toHaveLength(1);

    wts[0].rejectClosed();
    await vi.advanceTimersByTimeAsync(0);
    expect(transport.phase).toMatchObject({
      phase: "reconnecting",
      attempt: 1,
      retryAtMs: Date.now() + 500,
    });
    expect(wts[0].close).toHaveBeenCalled();

    await vi.advanceTimersByTimeAsync(500);
    expect(transport.phase).toEqual({ phase: "connected" });
    expect(wts).toHaveLength(2);
    expect(fetchInfo).toHaveBeenCalledTimes(2);

    wts[1].rejectClosed();
    await vi.advanceTimersByTimeAsync(0);
    // Backoff reset: the delay after a successful connection is 500 ms again.
    expect(transport.phase).toMatchObject({
      phase: "reconnecting",
      attempt: 1,
      retryAtMs: Date.now() + 500,
    });
    transport.stop();
  });

  it("surfaces the server's close reason in the reconnecting phase", async () => {
    const wts: FakeWt[] = [];
    const transport = makeTransport({
      fetchInfo: () => Promise.resolve(INFO),
      createWebTransport: () => {
        const wt = makeFakeWt();
        wts.push(wt);
        return wt;
      },
    });
    transport.start();
    await vi.advanceTimersByTimeAsync(0);

    // The relay kicks with a WebTransportCloseInfo; its reason must reach the UI.
    wts[0].resolveClosed({ closeCode: 1, reason: "reliable channel overflow" });
    await vi.advanceTimersByTimeAsync(0);
    expect(transport.phase).toMatchObject({
      phase: "reconnecting",
      reason: "reliable channel overflow",
    });

    // The next disconnect without a reason must not inherit the old one.
    await vi.advanceTimersByTimeAsync(500);
    expect(transport.phase).toEqual({ phase: "connected" });
    wts[1].resolveClosed(undefined);
    await vi.advanceTimersByTimeAsync(0);
    expect(transport.phase).toEqual({
      phase: "reconnecting",
      attempt: 1,
      retryAtMs: Date.now() + 500,
    });
    transport.stop();
  });

  it("reconnects when the session run ends even if the connection stays up", async () => {
    const fetchInfo = vi.fn(() => Promise.resolve(INFO));
    let endSession = () => {};
    const transport = makeTransport({
      fetchInfo,
      onSession: () =>
        new Promise<void>((resolve) => {
          endSession = resolve;
        }),
    });
    transport.start();

    await vi.advanceTimersByTimeAsync(0);
    expect(transport.phase).toEqual({ phase: "connected" });
    endSession();
    await vi.advanceTimersByTimeAsync(0);
    expect(transport.phase.phase).toBe("reconnecting");
    transport.stop();
  });

  it("stop() cancels a pending retry", async () => {
    const fetchInfo = vi.fn(() => Promise.reject(new Error("down")));
    const transport = makeTransport({ fetchInfo });
    transport.start();
    await vi.advanceTimersByTimeAsync(0);
    expect(fetchInfo).toHaveBeenCalledTimes(1);

    transport.stop();
    await vi.advanceTimersByTimeAsync(60_000);
    expect(fetchInfo).toHaveBeenCalledTimes(1);
  });

  it("fail() is terminal", async () => {
    const fetchInfo = vi.fn(() => Promise.reject(new Error("down")));
    const transport = makeTransport({ fetchInfo });
    transport.start();
    await vi.advanceTimersByTimeAsync(0);

    transport.fail("protocol mismatch");
    expect(transport.phase).toEqual({ phase: "failed", reason: "protocol mismatch" });
    await vi.advanceTimersByTimeAsync(60_000);
    expect(fetchInfo).toHaveBeenCalledTimes(1);
    expect(phases.at(-1)).toEqual({ phase: "failed", reason: "protocol mismatch" });
  });

  it("fails permanently on a protocol version mismatch from /api/info", async () => {
    const fetchInfo = vi.fn(() => Promise.resolve({ ...INFO, v: 99 }));
    const transport = makeTransport({ fetchInfo });
    transport.start();
    await vi.advanceTimersByTimeAsync(0);

    expect(transport.phase.phase).toBe("failed");
    await vi.advanceTimersByTimeAsync(60_000);
    expect(fetchInfo).toHaveBeenCalledTimes(1);
  });

  it("counts a failed WebTransport handshake as a failed attempt", async () => {
    const fetchInfo = vi.fn(() => Promise.resolve(INFO));
    const transport = makeTransport({
      fetchInfo,
      createWebTransport: () => makeFakeWt({ readyFails: true }),
    });
    transport.start();
    await vi.advanceTimersByTimeAsync(0);
    expect(transport.phase.phase).toBe("reconnecting");
    transport.stop();
  });

  it("stays in connecting after the QUIC handshake until sessionReady", async () => {
    const transport = makeTransport({
      fetchInfo: () => Promise.resolve(INFO),
      autoReady: false,
    });
    transport.start();
    await vi.advanceTimersByTimeAsync(0);
    // QUIC is up but no welcome arrived: the page is not usable yet.
    expect(transport.phase).toEqual({ phase: "connecting", attempt: 1 });

    transport.sessionReady();
    expect(transport.phase).toEqual({ phase: "connected" });
    transport.stop();
  });

  it("times out a hung /api/info fetch and falls back to backoff", async () => {
    const signals: AbortSignal[] = [];
    const transport = makeTransport({
      fetchInfo: (signal) =>
        new Promise((_, reject) => {
          signals.push(signal);
          signal.addEventListener("abort", () => reject(new Error("aborted")));
        }),
    });
    transport.start();
    await vi.advanceTimersByTimeAsync(0);
    expect(transport.phase).toEqual({ phase: "connecting", attempt: 1 });

    await vi.advanceTimersByTimeAsync(CONNECT_TIMEOUT_MS);
    expect(signals[0].aborted).toBe(true);
    expect(transport.phase).toMatchObject({ phase: "reconnecting", attempt: 1 });
    transport.stop();
  });

  it("times out a hung WebTransport handshake and closes the attempt", async () => {
    const wts: FakeWt[] = [];
    const transport = makeTransport({
      fetchInfo: () => Promise.resolve(INFO),
      createWebTransport: () => {
        const wt = makeFakeWt({ readyHangs: true });
        wts.push(wt);
        return wt;
      },
    });
    transport.start();
    await vi.advanceTimersByTimeAsync(0);
    expect(transport.phase).toEqual({ phase: "connecting", attempt: 1 });

    await vi.advanceTimersByTimeAsync(CONNECT_TIMEOUT_MS);
    expect(transport.phase).toMatchObject({ phase: "reconnecting", attempt: 1 });
    expect(wts[0].close).toHaveBeenCalled();
    transport.stop();
  });

  it("stop() aborts an in-flight connection attempt", async () => {
    const signals: AbortSignal[] = [];
    const transport = makeTransport({
      fetchInfo: (signal) =>
        new Promise((_, reject) => {
          signals.push(signal);
          signal.addEventListener("abort", () => reject(new Error("aborted")));
        }),
    });
    transport.start();
    await vi.advanceTimersByTimeAsync(0);

    transport.stop();
    expect(signals[0].aborted).toBe(true);
    await vi.advanceTimersByTimeAsync(60_000);
    expect(signals).toHaveLength(1);
    expect(phases.filter((p) => p.phase === "reconnecting")).toEqual([]);
  });
});
