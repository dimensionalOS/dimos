// Test harness: the relay end of one fake WebTransport connection, plus the
// spec/manifest builders and waiters shared by session.test.ts and
// react.test.tsx. Vitest-free on purpose so any test file can import it.

import {
  type ChannelSpec,
  ControlFrameReader,
  type DataFrame,
  DataFrameStreamReader,
  decodeDatagram,
  encodeControlFrame,
  encodeDataFrame,
  type Msg,
  type PanelSpec,
  PROTOCOL_VERSION,
  type RobotInfo,
  TOPIC_GUARD_STREAM_MAGIC,
  TOPIC_PUBLISH_STREAM_MAGIC,
  type TopicSubMsg,
  type TopicUpdateMsg,
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
  wtUrl: "https://127.0.0.1:1",
  certHash: "aGFzaA==",
  v: PROTOCOL_VERSION,
};

export const ROBOT_A: RobotInfo = { id: "a", name: "A", model: "go2" };
export const ROBOT_B: RobotInfo = { id: "b", name: "B", model: "go2" };

/** The relay end of one connection: collects viewer control messages and can
 * push control messages and data frames, so robot lifecycle transitions run
 * over the actual wire encoding while the "connection" stays up. */
export class FakeRelayEnd {
  /** Viewer control messages, liveness pings excluded. */
  readonly sent: Msg[] = [];
  readonly publications: DataFrame[] = [];
  readonly guards: DataFrame[] = [];
  /** Refuse this many publish/guard stream opens, like a relay that resets them. */
  failBinaryStreams = 0;
  pings = 0;
  aborted: number | undefined;
  answerPings = true;
  /** Decoded datagrams the viewer sent (the teleop twist path). */
  readonly sentDatagrams: Msg[] = [];
  /** Awaited per decoded viewer message: lets a test react at the exact
   * moment a message is observed while holding the session's control writer
   * (e.g. to land a data frame mid-sub-loop). */
  onMsg: ((msg: Msg) => void | Promise<void>) | null = null;
  readonly wt: WebTransportLike;
  #control!: ReadableStreamDefaultController<Uint8Array>;
  #uni!: ReadableStreamDefaultController<ReadableStream<Uint8Array>>;
  #topics = new Map<string, ReadableStreamDefaultController<Uint8Array>>();

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
          if (msg.t === "ping") {
            // The liveness ping: answered like the relay does (and kept out
            // of `sent`, which tests compare exactly) unless a test holds
            // the pongs back.
            this.pings += 1;
            if (this.answerPings) {
              this.push({
                t: "pong",
                n: msg.n,
                ts: msg.ts,
                ...(this.aborted === undefined ? {} : { aborted: this.aborted }),
              });
            }
            continue;
          }
          this.sent.push(msg);
          await this.onMsg?.(msg);
        }
      },
    });
    let closeWt = () => {};
    const closed = new Promise<unknown>((resolve) => {
      closeWt = () => resolve({});
    });
    let openedControl = false;
    this.wt = {
      ready: Promise.resolve(),
      closed,
      close: () => closeWt(),
      createBidirectionalStream: () => {
        if (!openedControl) {
          openedControl = true;
          return Promise.resolve({ readable, writable });
        }
        if (this.failBinaryStreams > 0) {
          this.failBinaryStreams -= 1;
          return Promise.reject(new Error("binary stream refused"));
        }
        const frames = new DataFrameStreamReader();
        let destination: DataFrame[] | null = null;
        return Promise.resolve({
          readable: new ReadableStream<Uint8Array>(),
          writable: new WritableStream<Uint8Array>({
            write: (bytes) => {
              if (destination === null) {
                if (bytes.byteLength !== 4) throw new Error("missing binary stream prefix");
                const magic = new DataView(bytes.buffer, bytes.byteOffset).getUint32(0, true);
                if (magic === TOPIC_PUBLISH_STREAM_MAGIC) destination = this.publications;
                else if (magic === TOPIC_GUARD_STREAM_MAGIC) destination = this.guards;
                else throw new Error("invalid binary stream prefix");
              } else destination.push(...frames.push(bytes));
            },
          }),
        });
      },
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
    const existing = this.#topics.get(ch);
    if (existing !== undefined) {
      existing.enqueue(frame);
      return;
    }
    this.#uni.enqueue(
      new ReadableStream<Uint8Array>({
        start: (c) => {
          c.enqueue(frame);
          if (ch.startsWith("@t:")) this.#topics.set(ch, c);
          else c.close();
        },
      }),
    );
  }

  endTopic(ch: string): void {
    this.#topics.get(ch)?.close();
    this.#topics.delete(ch);
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

  topicSubs(): TopicSubMsg[] {
    return this.sent.filter((m): m is TopicSubMsg => m.t === "topic_sub");
  }

  topicUpdates(): TopicUpdateMsg[] {
    return this.sent.filter((m): m is TopicUpdateMsg => m.t === "topic_update");
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
