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
import {
  type ChannelPolicy,
  readDataFrameBytes,
  readWebTransportPreamble,
  type ViewerSink,
} from "./forward.ts";
import type { Registry, RobotPeer, ViewerPeer } from "./registry.ts";

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
  /** Close reason; set before the transport close so a hello resend still
   * queued on this session cannot re-register it after a takeover. */
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

  close(reason: string): void {
    this.closed = reason;
    try {
      this.#wt.close({ closeCode: 0, reason });
    } catch {
      // already gone
    }
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
      // Robot-leg control rides datagrams: aioquic dies if the relay writes
      // on robot-opened bidi streams, so hello/welcome/subs/ping/pong live here.
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
        this.sendMsg({
          t: "error",
          code: "version_mismatch",
          message: `protocol v${PROTOCOL_VERSION} required, got v${msg.v}`,
        });
        closeAfterFlush(this.#wt, "version mismatch");
        return false;
      }
      if (msg.role !== "robot") {
        this.sendMsg({
          t: "error",
          code: "role_mismatch",
          message: "the /robot endpoint requires role=robot",
        });
        closeAfterFlush(this.#wt, "role mismatch");
        return false;
      }
      if (msg.robot === undefined) {
        this.sendMsg({
          t: "error",
          code: "missing_robot_id",
          message: "robot hello must carry robot{id,name,model}",
        });
        closeAfterFlush(this.#wt, "missing robot id");
        return false;
      }
      // First hello wins; resends (the bridge repeats hello until welcome)
      // must not mutate identity mid-session.
      if (this.info === null) {
        this.info = msg.robot;
        this.channels = msg.manifest?.channels ?? [];
      }
      this.sendMsg({ t: "welcome", v: PROTOCOL_VERSION });
      this.#registry.registerRobot(this);
    } else if (msg.t === "ping") {
      this.sendMsg({ t: "pong", n: msg.n, ts: msg.ts });
    }
    return true;
  }

  #frameLoop(): void {
    (async () => {
      // Data frames arrive on one-shot bidi streams (Deno never delivers
      // incoming uni payloads), accepted at the QUIC level rather than via
      // wt.incomingBidirectionalStreams: a reset racing that iterator's
      // internal preamble read errors it permanently and would silently end
      // this loop, while the raw accept only fails with the connection.
      // Abort our send half: RESET is invisible to aioquic's h3 layer and
      // releases stream credit; a FIN would kill it.
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
    let sendOrder = 1;
    this.sink = {
      async sendFrame(bytes: Uint8Array): Promise<void> {
        // waitUntilAvailable: a slow page exhausts stream credit; without it
        // this throws and we would drop a live viewer. Decreasing sendOrder
        // keeps stream completions FIFO on the wire (quinn round-robins
        // otherwise and frames complete in ~1 s waves).
        const stream = await wt.createUnidirectionalStream({
          waitUntilAvailable: true,
          sendOrder: -(sendOrder++),
        });
        const writer = stream.getWriter();
        await writer.write(bytes);
        await writer.close();
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
    if (msg.t === "hello") this.#push = reply;
    if (!this.#registry.onViewerMsg(this, msg, reply)) {
      closeAfterFlush(this.#wt, "version mismatch");
      return false;
    }
    return true;
  }

  #streamLoop(): void {
    (async () => {
      // Browser-leg control: viewer-opened bidi stream, replies on the same
      // stream. Deno may write on viewer-initiated streams (browsers are not
      // aioquic). wt.incomingBidirectionalStreams is safe here: unlike the
      // robot leg, viewers never reset a stream racing its acceptance.
      for await (const bidi of this.#wt.incomingBidirectionalStreams) {
        (async () => {
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
      // Datagram control for viewers too: browsers use the bidi stream above,
      // but the Python test viewer cannot receive replies on its own bidi
      // streams (aioquic), so the whole session flow works over datagrams on
      // both legs. The relay answers pings itself (RTT works with no robot
      // connected); teleop routing arrives in T6.
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
