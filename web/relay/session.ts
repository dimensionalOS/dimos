// Per-connection session objects: handshake, control loops, and (robot leg)
// the raw-QUIC data stream loop. Sessions own transport quirks; all routing
// and subscription policy lives in registry.ts.
//
// Leg asymmetry, forced by upstream bugs (see web/README.md):
// - Robot (aioquic): control = datagrams both ways (the relay must never
//   write on robot-opened bidi streams); data = one-shot bidi streams the
//   relay never writes on (send half aborted with RESET, never FIN).
// - Viewer (browser): control = viewer-opened bidi stream (replies + pushes
//   on the same stream) or datagrams (Python test viewer); data = relay-
//   opened uni streams.
import {
  type ChannelSpec,
  ControlFrameReader,
  decodeDatagram,
  encodeControlFrame,
  encodeDatagram,
  type Msg,
  PROTOCOL_VERSION,
  type RobotInfo,
} from "@dimos/shared";
import { parseManifest } from "@dimos/shared/manifest";
import {
  type ChannelPolicy,
  type FrameWriter,
  readDataFrameBytes,
  readWebTransportPreamble,
  type ViewerSink,
} from "./forward.ts";
import type { Registry, RobotPeer, ViewerPeer } from "./registry.ts";

// Relay->viewer send priorities, all in one place. WebTransport sendOrder
// (backed by quinn stream priority here): queued bytes of a higher-order
// stream are sent before those of a lower-order one, so control must outrank
// reliable telemetry and reliable must outrank latest video, whose per-frame
// streams count down from -1 to complete oldest-first (README bug 7).
// Datagrams (the Python viewer's control leg) have no sendOrder API.
export const CONTROL_SEND_ORDER = 2;
export const RELIABLE_SEND_ORDER = 1;

function closeAfterFlush(wt: WebTransport, reason: string): void {
  // Session close discards queued stream/datagram data, so give a just-sent
  // reply (e.g. the version_mismatch error) a moment to reach the wire.
  setTimeout(() => {
    try {
      wt.close({ closeCode: 1, reason });
    } catch {
      // already gone
    }
  }, 250);
}

export class RobotSession implements RobotPeer {
  info: RobotInfo | null = null;
  channels: ChannelSpec[] = [];
  /** Close reason; set before transport close so rejected hello resends
   * cannot register this session. */
  closed: string | null = null;
  readonly #wt: WebTransport;
  readonly #conn: Deno.QuicConn;
  readonly #registry: Registry;
  readonly #dgWriter: WritableStreamDefaultWriter<Uint8Array>;

  constructor(wt: WebTransport, conn: Deno.QuicConn, registry: Registry) {
    this.#wt = wt;
    this.#conn = conn;
    this.#registry = registry;
    this.#dgWriter = wt.datagrams.writable.getWriter();
  }

  sendMsg(msg: Msg): void {
    this.#dgWriter.write(encodeDatagram(msg)).catch(() => {});
  }

  #reject(code: string, message: string, reason: string): false {
    this.sendMsg({ t: "error", code, message });
    this.closed = reason;
    closeAfterFlush(this.#wt, reason);
    return false;
  }

  start(): void {
    console.log("[relay] robot connected");
    this.#wt.closed
      .catch(() => {})
      .finally(() => this.#registry.robotClosed(this));
    this.#controlLoop();
    this.#frameLoop();
  }

  #controlLoop(): void {
    (async () => {
      for await (const dg of this.#wt.datagrams.readable) {
        const msg = decodeDatagram(dg);
        if (msg === null) continue;
        if (!this.#onControlMsg(msg)) return;
      }
    })().catch(() => {});
  }

  /** Replies to hello/ping; returns false once the session is being closed. */
  #onControlMsg(msg: Msg): boolean {
    if (msg.t === "hello") {
      if (msg.v !== PROTOCOL_VERSION) {
        return this.#reject(
          "version_mismatch",
          `protocol v${PROTOCOL_VERSION} required, got v${msg.v}`,
          "version mismatch",
        );
      }
      if (msg.role !== "robot") {
        return this.#reject(
          "role_mismatch",
          "the /robot endpoint requires role=robot",
          "role mismatch",
        );
      }
      if (msg.robot === undefined) {
        return this.#reject(
          "missing_robot_id",
          "robot hello must carry robot{id,name,model}",
          "missing robot id",
        );
      }
      // Manifest-less hellos are legal (transport tests); a declared manifest
      // must pass the domain rules or duplicate/bogus channels would be
      // interpreted inconsistently downstream.
      if (msg.manifest !== undefined) {
        try {
          parseManifest(msg.manifest);
        } catch (e) {
          return this.#reject("invalid_manifest", (e as Error).message, "invalid manifest");
        }
      }
      // First hello wins; resends (the bridge repeats hello until welcome)
      // must not mutate identity mid-session.
      if (this.info === null) {
        this.info = msg.robot;
        this.channels = msg.manifest?.channels ?? [];
      }
      if (!this.#registry.registerRobot(this)) {
        return this.#reject(
          "robot_id_conflict",
          `robot id ${msg.robot.id} already has a live session`,
          "robot id conflict",
        );
      }
      this.sendMsg({ t: "welcome", v: PROTOCOL_VERSION });
    } else if (msg.t === "ping") {
      this.sendMsg({ t: "pong", n: msg.n, ts: msg.ts });
    }
    return true;
  }

  #frameLoop(): void {
    (async () => {
      for await (const bidi of this.#conn.incomingBidirectionalStreams) {
        bidi.writable.abort().catch(() => {});
        (async () => {
          await readWebTransportPreamble(bidi.readable);
          this.#registry.onRobotFrame(this, await readDataFrameBytes(bidi.readable));
        })().catch(() => {
          // reset before/mid-frame (stale latest-wins write): drop the partial
        });
      }
    })().catch((e) => {
      console.log("[relay] robot stream loop ended:", (e as Error)?.message ?? e);
    });
  }
}

export class ViewerSession implements ViewerPeer {
  readonly id: number;
  watched: string | null = null;
  readonly subs = new Set<string>();
  readonly policies = new Map<string, ChannelPolicy>();
  greeted = false;
  readonly sink: ViewerSink;
  readonly #wt: WebTransport;
  readonly #registry: Registry;
  /** Push channel for robots events, chosen by whichever leg carried hello. */
  #push: ((msg: Msg) => void) | null = null;

  constructor(wt: WebTransport, id: number, registry: Registry) {
    this.#wt = wt;
    this.id = id;
    this.#registry = registry;
    let latestOrder = 1;
    this.sink = {
      async sendFrame(bytes: Uint8Array): Promise<void> {
        const stream = await wt.createUnidirectionalStream({
          waitUntilAvailable: true,
          sendOrder: -(latestOrder++),
        });
        const writer = stream.getWriter();
        await writer.write(bytes);
        await writer.close();
      },
      async openStream(): Promise<FrameWriter> {
        // Persistent stream for a reliable channel.
        const stream = await wt.createUnidirectionalStream({
          waitUntilAvailable: true,
          sendOrder: RELIABLE_SEND_ORDER,
        });
        return stream.getWriter();
      },
      kick(reason: string): void {
        console.log(`[relay] kicking viewer: ${reason}`);
        try {
          wt.close({ closeCode: 1, reason });
        } catch {
          // already gone
        }
      },
    };
  }

  sendMsg(msg: Msg): void {
    this.#push?.(msg);
  }

  start(): void {
    this.#registry.addViewer(this);
    console.log(`[relay] viewer ${this.id} connected`);
    this.#wt.closed
      .catch(() => {})
      .finally(() => {
        this.#registry.viewerClosed(this);
        console.log(`[relay] viewer ${this.id} disconnected`);
      });
    this.#streamLoop();
    this.#datagramLoop();
  }

  #dispatch(msg: Msg, reply: (msg: Msg) => void): boolean {
    if (!this.#registry.onViewerMsg(this, msg, reply)) {
      closeAfterFlush(this.#wt, "viewer handshake rejected");
      return false;
    }
    if (msg.t === "hello" && this.greeted) this.#push = reply;
    return true;
  }

  #streamLoop(): void {
    (async () => {
      for await (const bidi of this.#wt.incomingBidirectionalStreams) {
        (async () => {
          // Viewer-opened, so its send half starts at the default order 0,
          // which a saturated reliable stream would starve.
          bidi.writable.sendOrder = CONTROL_SEND_ORDER;
          const writer = bidi.writable.getWriter();
          const reply = (m: Msg) => {
            writer.write(encodeControlFrame(m)).catch(() => {});
          };
          const frames = new ControlFrameReader();
          for await (const chunk of bidi.readable) {
            for (const msg of frames.push(chunk)) {
              if (!this.#dispatch(msg, reply)) return;
            }
          }
          writer.releaseLock();
        })().catch((e) =>
          console.log("[relay] viewer control stream ended:", (e as Error)?.message ?? e)
        );
      }
    })().catch(() => {});
  }

  #datagramLoop(): void {
    const dgWriter = this.#wt.datagrams.writable.getWriter();
    (async () => {
      const reply = (m: Msg) => {
        dgWriter.write(encodeDatagram(m)).catch(() => {});
      };
      for await (const dg of this.#wt.datagrams.readable) {
        const msg = decodeDatagram(dg);
        if (msg === null) continue;
        if (!this.#dispatch(msg, reply)) return;
      }
    })().catch(() => {});
  }
}
