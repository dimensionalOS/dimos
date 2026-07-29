// Viewer session: drives the control stream (hello, robots, watch, manifest,
// sub) and the incoming uni-stream data plane on top of ReconnectingTransport,
// writing everything into the stores. One instance lives for the page.

import {
  type ChannelSpec,
  ControlFrameReader,
  type DataFrame,
  DataFrameStreamError,
  DataFrameStreamReader,
  encodeControlFrame,
  type Msg,
  PROTOCOL_VERSION,
  type RobotInfo,
} from "@dimos/shared";
import { parseManifest } from "@dimos/shared/manifest";
import { getDecoder } from "./decoders/index.ts";
import { ChannelStore, StatusStore } from "./store.ts";
import { ReconnectingTransport, type TransportDeps, type WebTransportLike } from "./transport.ts";

const UI_TICK_MS = 500;

export interface SessionHandle {
  status: StatusStore;
  channels: ChannelStore;
  stop(): void;
}

/** True when both lists describe the same channels (order-insensitive). */
export function manifestsEqual(a: ChannelSpec[], b: ChannelSpec[]): boolean {
  if (a.length !== b.length) return false;
  const key = (c: ChannelSpec) => c.ch;
  const sortedA = [...a].sort((x, y) => key(x).localeCompare(key(y)));
  const sortedB = [...b].sort((x, y) => key(x).localeCompare(key(y)));
  return sortedA.every((c, i) => {
    const other = sortedB[i];
    return (
      c.ch === other.ch &&
      c.encoding === other.encoding &&
      c.delivery === other.delivery &&
      c.maxHz === other.maxHz
    );
  });
}

/** Local auto-select policy: watch the robot only when it is the only one. */
export function pickAutoWatch(robots: RobotInfo[]): RobotInfo | null {
  return robots.length === 1 ? robots[0] : null;
}

/**
 * Channels worth subscribing: only those with a decoder. Subscribing to
 * undecodable channels wastes encode CPU and bandwidth, and a 15 Hz JPEG
 * stream nobody renders overflows the relay's reliable FIFO under Firefox's
 * tighter QUIC credit (the relay kicks the viewer every ~8 s). Panels take
 * over subscription decisions in T7; the video channel joins in T5 with its
 * decoder.
 */
export function subscribableChannels(channels: ChannelSpec[]): ChannelSpec[] {
  return channels.filter((spec) => getDecoder(spec.encoding) !== undefined);
}

class Session {
  readonly status = new StatusStore();
  readonly channels = new ChannelStore();
  readonly transport: ReconnectingTransport;

  // Bumped per connection; data-plane writes from a previous connection's
  // still-draining reader loops are dropped by comparing against it.
  #runId = 0;
  #manifest: ChannelSpec[] | null = null;
  #ticker: ReturnType<typeof setInterval>;

  constructor(transportDeps: TransportDeps = {}) {
    this.transport = new ReconnectingTransport(
      {
        onPhase: (phase) => this.status.update({ transport: phase }),
        onSession: (wt) => this.#runSession(wt),
      },
      transportDeps,
    );
    this.#ticker = setInterval(() => this.channels.publishUi(), UI_TICK_MS);
  }

  stop(): void {
    clearInterval(this.#ticker);
    this.transport.stop();
  }

  async #runSession(wt: WebTransportLike): Promise<void> {
    const runId = ++this.#runId;
    const control = await wt.createBidirectionalStream();
    const writer = control.writable.getWriter();
    const send = async (msg: Msg) => {
      await writer.write(encodeControlFrame(msg));
    };
    await send({ t: "hello", v: PROTOCOL_VERSION, role: "viewer" });
    void this.#readUniStreams(wt, runId);

    const reader = control.readable.getReader();
    const frames = new ControlFrameReader();
    try {
      while (true) {
        const { value, done } = await reader.read();
        if (done) break;
        for (const msg of frames.push(value)) {
          switch (msg.t) {
            case "welcome":
              // Session-level success: only now is the page usable, so only
              // now the transport may show "connected".
              this.transport.sessionReady();
              this.status.update({ lastError: null });
              break;
            case "robots": {
              const pick = pickAutoWatch(msg.robots);
              this.status.update({ robot: pick, robotCount: msg.robots.length });
              if (pick === null) {
                this.#clearProducer();
              } else {
                // Unconditional (and idempotent on the relay): a same-id
                // robot restart is announced with the id we already watch,
                // so only a fresh watch re-confirms it, refreshes the
                // manifest, and retries after an unknown_robot race.
                await send({ t: "watch", robotId: pick.id });
              }
              break;
            }
            case "manifest": {
              // A reply to a watch that raced a robot change must not be
              // adopted: only the currently picked robot's manifest counts.
              if (msg.robotId !== this.status.get().robot?.id) break;
              let channels: ChannelSpec[];
              try {
                // Domain validation (duplicate/bogus ids) on top of the
                // transport shape check: a duplicate id would make the
                // store and the channel list disagree on the winner.
                channels = parseManifest({ channels: msg.channels }).channels;
              } catch (e) {
                this.status.update({ lastError: `invalid manifest: ${(e as Error).message}` });
                break;
              }
              this.status.update({ lastError: null });
              for (const spec of subscribableChannels(channels)) {
                await send({ t: "sub", ch: spec.ch });
              }
              this.#applyManifest(channels);
              break;
            }
            case "error": {
              if (msg.code === "version_mismatch") {
                this.transport.fail(msg.message);
              } else {
                this.status.update({ lastError: `${msg.code}: ${msg.message}` });
              }
              break;
            }
            default:
              // pong, robot-side messages: nothing to do
              break;
          }
        }
      }
    } catch {
      // control stream died with the connection; the transport reconnects
    }
  }

  /**
   * Adopt a confirmed manifest. The producer behind it may differ from the
   * previous one (viewer reconnect, robot restart or replacement), so seq
   * tracking always rebaselines; a first adopt drops data left over from a
   * dead producer, and a changed manifest additionally remounts.
   */
  #applyManifest(channels: ChannelSpec[]): void {
    const prev = this.#manifest;
    this.#manifest = channels;
    if (prev === null) {
      this.channels.reset();
      this.status.update({ channels });
    } else if (!manifestsEqual(prev, channels)) {
      this.channels.reset();
      this.status.update({ channels, epoch: this.status.get().epoch + 1 });
    } else {
      this.status.update({ channels });
    }
    this.channels.rebaseline();
  }

  /**
   * Zero or ambiguous robots: the watch is no longer confirmed. Drop the
   * manifest and all channel data and remount, so nothing stale survives
   * under whatever robot is confirmed next.
   */
  #clearProducer(): void {
    if (this.#manifest === null) return;
    this.#manifest = null;
    this.channels.reset();
    this.status.update({ channels: [], epoch: this.status.get().epoch + 1 });
  }

  async #readUniStreams(wt: WebTransportLike, runId: number): Promise<void> {
    const streams = wt.incomingUnidirectionalStreams.getReader();
    try {
      while (true) {
        const { value, done } = await streams.read();
        if (done) break;
        void this.#readStreamFrames(value, runId);
      }
    } catch {
      // connection died; the transport reconnects
    }
  }

  // A latest stream carries one frame; a reliable channel's persistent stream
  // carries them back to back. Frames dispatch on byte count (the relay's FIN
  // can be seconds late); the stream is never cancelled - reading to its end
  // costs nothing and a cancel would reset the persistent stream.
  async #readStreamFrames(stream: ReadableStream<Uint8Array>, runId: number): Promise<void> {
    const reader = stream.getReader();
    const frames = new DataFrameStreamReader();
    try {
      while (true) {
        const { value, done } = await reader.read();
        if (done) break;
        for (const frame of frames.push(value)) {
          if (runId === this.#runId) this.#ingest(frame);
        }
      }
    } catch (e) {
      // reset/aborted stream: a partial latest-wins frame is dropped by
      // design. Framing corruption still delivers the frames decoded before
      // it; only the corrupt stream is abandoned.
      if (e instanceof DataFrameStreamError && runId === this.#runId) {
        for (const frame of e.frames) this.#ingest(frame);
      }
    }
  }

  #ingest(frame: DataFrame): void {
    // No adopted manifest means no confirmed producer: anything arriving is
    // stale drain from a dead robot session and must not re-dirty the store.
    if (this.#manifest === null) return;
    const spec = this.#manifest.find((c) => c.ch === frame.header.ch);
    const decoder = getDecoder(spec?.encoding);
    let value: unknown;
    let preview: string | undefined;
    let decodeOk = true;
    if (decoder !== undefined) {
      try {
        ({ value, preview } = decoder(frame.payload, frame.header));
      } catch {
        decodeOk = false;
      }
    }
    this.channels.ingest(frame.header.ch, frame.header, value, decodeOk, preview);
  }
}

export function startSession(transportDeps: TransportDeps = {}): SessionHandle {
  const session = new Session(transportDeps);
  session.transport.start();
  return {
    status: session.status,
    channels: session.channels,
    stop: () => session.stop(),
  };
}
