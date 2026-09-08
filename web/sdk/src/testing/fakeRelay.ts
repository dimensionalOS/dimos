// Test harness: the relay end of one fake WebTransport connection, plus the
// spec/manifest builders and waiters shared by session.test.ts and
// react.test.tsx. Vitest-free on purpose so any test file can import it.

import {
  type ChannelSpec,
  ControlFrameReader,
  decodeDatagram,
  encodeControlFrame,
  encodeDataFrame,
  type Msg,
  type PanelSpec,
  PROTOCOL_VERSION,
  type RobotInfo,
} from "@dimos/shared";
import type { Manifest } from "@dimos/shared/manifest";
import type { RelayInfo, WebTransportLike } from "../transport.ts";

// Normalized specs on purpose: pushing them over the fake wire and parsing
// them back yields the identical objects, so adoption asserts stay exact.
export function spec(over: Partial<ChannelSpec> = {}): ChannelSpec {
  return {
    ch: "odom",
    dir: "rx",
    encoding: "pose.json.v1",
    delivery: "reliable",
    maxHz: 20,
    params: {},
    publish: "none",
    requiredScope: null,
    ...over,
  };
}

export function panel(over: Partial<PanelSpec> & { id: string; kind: string }): PanelSpec {
  return { title: "", channels: [], params: {}, ...over };
}

export function manifest(
  channels: ChannelSpec[],
  panels: PanelSpec[] = [],
  layout: Manifest["layout"] = null,
  pages: string[] = [],
): Manifest {
  return { version: 1, channels, panels, layout, pages };
}

export const INFO: RelayInfo = {
  wtUrl: "https://127.0.0.1:1/viewer",
  certHash: "aGFzaA==",
  v: PROTOCOL_VERSION,
};

export const ROBOT_A: RobotInfo = { id: "a", name: "A", model: "go2" };
export const ROBOT_B: RobotInfo = { id: "b", name: "B", model: "go2" };

/** The relay end of one connection: collects viewer control messages and can
 * push control messages and data frames, so robot lifecycle transitions run
 * over the actual wire encoding while the "connection" stays up. */
export class FakeRelayEnd {
  readonly sent: Msg[] = [];
  /** Decoded datagrams the viewer sent (the teleop twist path). */
  readonly sentDatagrams: Msg[] = [];
  /** Awaited per decoded viewer message: lets a test react at the exact
   * moment a message is observed while holding the session's control writer
   * (e.g. to land a data frame mid-sub-loop). */
  onMsg: ((msg: Msg) => void | Promise<void>) | null = null;
  readonly wt: WebTransportLike;
  #control!: ReadableStreamDefaultController<Uint8Array>;
  #uni!: ReadableStreamDefaultController<ReadableStream<Uint8Array>>;

  constructor() {
    const inbound = new ControlFrameReader();
    const readable = new ReadableStream<Uint8Array>({
      start: (c) => {
        this.#control = c;
      },
    });
    const writable = new WritableStream<Uint8Array>({
      write: async (chunk) => {
        for (const msg of inbound.push(chunk)) {
          this.sent.push(msg);
          await this.onMsg?.(msg);
        }
      },
    });
    let closeWt = () => {};
    const closed = new Promise<unknown>((resolve) => {
      closeWt = () => resolve({});
    });
    this.wt = {
      ready: Promise.resolve(),
      closed,
      close: () => closeWt(),
      createBidirectionalStream: () => Promise.resolve({ readable, writable }),
      incomingUnidirectionalStreams: new ReadableStream<ReadableStream<Uint8Array>>({
        start: (c) => {
          this.#uni = c;
        },
      }),
      datagrams: {
        writable: new WritableStream<Uint8Array>({
          write: (chunk) => {
            const msg = decodeDatagram(chunk);
            if (msg !== null) this.sentDatagrams.push(msg);
          },
        }),
      },
    };
  }

  push(msg: Msg): void {
    this.#control.enqueue(encodeControlFrame(msg));
  }

  pushManifest(robotId: string, m: Manifest): void {
    this.push({ t: "manifest", robotId, manifest: m as unknown as Record<string, unknown> });
  }

  /** One data frame on its own uni stream, JSON payload like the bridge's. */
  pushFrame(seq: number, value: unknown, ch = "odom"): void {
    this.pushRaw(seq, new TextEncoder().encode(JSON.stringify(value)), ch);
  }

  /** Several back-to-back frames on one uni stream (a reliable channel's
   * persistent stream), JSON payloads like the bridge's. */
  pushFrames(entries: { seq: number; value: unknown }[], ch = "odom"): void {
    this.#uni.enqueue(
      new ReadableStream<Uint8Array>({
        start: (c) => {
          for (const { seq, value } of entries) {
            c.enqueue(encodeDataFrame(
              { ch, seq, ts: seq, delivery: "reliable" },
              new TextEncoder().encode(JSON.stringify(value)),
            ));
          }
          c.close();
        },
      }),
    );
  }

  /** One data frame on its own uni stream, arbitrary payload bytes. */
  pushRaw(seq: number, payload: Uint8Array, ch: string, meta?: Record<string, unknown>): void {
    const frame = encodeDataFrame({ ch, seq, ts: seq, delivery: "reliable", meta }, payload);
    this.#uni.enqueue(
      new ReadableStream<Uint8Array>({
        start: (c) => {
          c.enqueue(frame);
          c.close();
        },
      }),
    );
  }

  /** End the control stream (the session's read loop sees done and treats
   * the connection as dead; the transport then reconnects). */
  endControl(): void {
    this.#control.close();
  }

  watches(id: string): number {
    return this.sent.filter((m) => m.t === "watch" && m.robotId === id).length;
  }

  pubs(): Msg[] {
    return this.sent.filter((m) => m.t === "pub");
  }

  subs(): string[] {
    return this.sent.flatMap((m) => (m.t === "sub" ? [m.ch] : []));
  }

  unsubs(): string[] {
    return this.sent.flatMap((m) => (m.t === "unsub" ? [m.ch] : []));
  }
}

export async function until(cond: () => boolean, what = "condition"): Promise<void> {
  const deadline = Date.now() + 2000;
  while (!cond()) {
    if (Date.now() > deadline) throw new Error(`timed out waiting for ${what}`);
    await new Promise((resolve) => setTimeout(resolve, 2));
  }
}

/** Lets already-queued streams/messages drain before a negative assertion. */
export function settle(): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, 20));
}
