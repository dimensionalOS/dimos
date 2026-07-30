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
  type PanelSpec,
  PROTOCOL_VERSION,
  type RobotInfo,
} from "@dimos/shared";
import { type Manifest, parseManifest } from "@dimos/shared/manifest";
import { getPanel } from "../panels/registry.ts";
import { getDecoder } from "./decoders/index.ts";
import { ChannelStore, StatusStore } from "./store.ts";
import { ReconnectingTransport, type TransportDeps, type WebTransportLike } from "./transport.ts";

const UI_TICK_MS = 500;

export interface SessionHandle {
  status: StatusStore;
  channels: ChannelStore;
  stop(): void;
}

/** True when both manifests describe the same channels (order-insensitive)
 * and the same panels (order-sensitive: panel order is display order). */
export function manifestsEqual(a: Manifest, b: Manifest): boolean {
  if (a.channels.length !== b.channels.length || a.panels.length !== b.panels.length) {
    return false;
  }
  const key = (c: ChannelSpec) => c.ch;
  const sortedA = [...a.channels].sort((x, y) => key(x).localeCompare(key(y)));
  const sortedB = [...b.channels].sort((x, y) => key(x).localeCompare(key(y)));
  const channelsEqual = sortedA.every((c, i) => {
    const other = sortedB[i];
    return (
      c.ch === other.ch &&
      c.encoding === other.encoding &&
      c.delivery === other.delivery &&
      c.maxHz === other.maxHz
    );
  });
  return channelsEqual && a.panels.every((p, i) => {
    const other = b.panels[i];
    return (
      p.id === other.id &&
      p.kind === other.kind &&
      p.channels.length === other.channels.length &&
      p.channels.every((ch, j) => ch === other.channels[j])
    );
  });
}

/** Local auto-select policy: watch the robot only when it is the only one. */
export function pickAutoWatch(robots: RobotInfo[]): RobotInfo | null {
  return robots.length === 1 ? robots[0] : null;
}

// Encodings whose subscription costs real encode CPU and bandwidth;
// subscribed only when a panel this build can render binds them. T7 moves
// all subscription decisions to panels.
const PANEL_ONLY_ENCODINGS = new Set(["jpeg.v1", "costmap.zlib.v1"]);

/** True when this build can put the channel to use: it has a decoder, and a
 * panel-only encoding is additionally bound by a renderable panel. */
export function channelSubscribable(spec: ChannelSpec, panels: PanelSpec[]): boolean {
  if (getDecoder(spec.encoding) === undefined) return false;
  if (!PANEL_ONLY_ENCODINGS.has(spec.encoding)) return true;
  return panels.some((p) => getPanel(p.kind) !== undefined && p.channels.includes(spec.ch));
}

/**
 * Channels worth subscribing: only those with a decoder, and panel-only
 * encodings only when a renderable panel binds them. Subscribing to channels
 * nobody can render wastes encode CPU and bandwidth, and a high-rate JPEG stream
 * nobody renders overflows the relay's reliable FIFO under Firefox's tighter
 * QUIC credit (the relay kicks the viewer every ~8 s). Panels take over all
 * subscription decisions in T7.
 */
export function subscribableChannels(channels: ChannelSpec[], panels: PanelSpec[]): ChannelSpec[] {
  return channels.filter((spec) => channelSubscribable(spec, panels));
}

class Session {
  readonly status = new StatusStore();
  readonly channels = new ChannelStore();
  readonly transport: ReconnectingTransport;

  // Bumped per connection; data-plane writes from a previous connection's
  // still-draining reader loops are dropped by comparing against it.
  #runId = 0;
  #manifest: Manifest | null = null;
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
              let manifest: Manifest;
              try {
                // Domain validation (duplicate/bogus ids, panel channel refs)
                // on top of the transport shape check: a duplicate id would
                // make the store and the channel list disagree on the winner.
                manifest = parseManifest({ channels: msg.channels, panels: msg.panels });
              } catch (e) {
                this.status.update({ lastError: `invalid manifest: ${(e as Error).message}` });
                break;
              }
              this.status.update({ lastError: null });
              // Adopt before subscribing: a sub can trigger an immediate
              // frame (the bridge replays the cached costmap), and #ingest
              // drops everything while no manifest is adopted.
              this.#applyManifest(manifest);
              for (const spec of subscribableChannels(manifest.channels, manifest.panels)) {
                await send({ t: "sub", ch: spec.ch });
              }
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
  #applyManifest(manifest: Manifest): void {
    const prev = this.#manifest;
    this.#manifest = manifest;
    const patch = { channels: manifest.channels, panels: manifest.panels };
    if (prev === null) {
      this.channels.reset();
      this.status.update(patch);
    } else if (!manifestsEqual(prev, manifest)) {
      this.channels.reset();
      this.status.update({ ...patch, epoch: this.status.get().epoch + 1 });
    } else {
      this.status.update(patch);
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
    this.status.update({ channels: [], panels: [], epoch: this.status.get().epoch + 1 });
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
    const spec = this.#manifest.channels.find((c) => c.ch === frame.header.ch);
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
