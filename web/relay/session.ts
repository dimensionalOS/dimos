// Per-connection session objects: handshake, control loops, and (robot leg)
// the raw-QUIC data stream loop. Sessions own transport quirks; all routing
// and subscription policy lives in registry.ts.
//
// Leg asymmetry, forced by upstream bugs (see web/README.md):
// - Robot (aioquic): robot->relay hello rides an @control data frame on a
//   one-shot bidi stream (v5); every relay->robot control (welcome, errors,
//   pong, subs, teleop) rides datagrams, because the relay must never write
//   on robot-opened bidi streams (their send half is aborted with RESET,
//   never FIN). Data = one-shot bidi streams under the same rule.
// - Viewer (browser): control = viewer-opened bidi stream (replies + pushes
//   on the same stream) or datagrams (Python test viewer); data = relay-
//   opened uni streams.
import {
  type ChannelSpec,
  CONTROL_CHANNEL,
  ControlFrameReader,
  decodeDatagram,
  encodeControlFrame,
  encodeDatagram,
  type FrameHeader,
  type HelloMsg,
  type Msg,
  peekDataFrameLengths,
  PROTOCOL_VERSION,
  type RobotInfo,
  type RobotManifest,
} from "@dimos/shared";
import { parseManifest } from "@dimos/shared/manifest";
import {
  type ChannelPolicy,
  ControlPayloadTooLargeError,
  type FrameSend,
  type FrameWriter,
  readRobotFrame,
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

// Application error code carried by latest-stream resets (mirrors the Python
// robot leg's STALE_STREAM_ERROR_CODE). Receivers do not act on it; it only
// labels the reset for debugging.
export const STALE_STREAM_ERROR_CODE = 0x01;

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
  /** Normalized channel specs (parseManifest output): feeds the delivery map
   * and sub validation. */
  channels: ChannelSpec[] = [];
  /** The manifest exactly as the robot sent it: forwarded verbatim to
   * viewers, never normalized (the relay stays layout-blind). */
  manifest: RobotManifest | null = null;
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
    // Concurrent stream tasks can race a rejection; only the first wins.
    if (this.closed !== null) return false;
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

  /** Replies to pings; rejects datagram hello (v5 moved the robot hello to
   * an @control stream frame, so a datagram hello means a v4-or-older
   * bridge). Returns false once the session is being closed. */
  #onControlMsg(msg: Msg): boolean {
    if (msg.t === "hello") {
      return this.#reject(
        "version_mismatch",
        `protocol v${PROTOCOL_VERSION} robot hello rides an @control stream frame; ` +
          "datagram hello is v4 or older",
        "datagram hello",
      );
    }
    if (msg.t === "ping") this.sendMsg({ t: "pong", n: msg.n, ts: msg.ts });
    return true;
  }

  /** One @-channel frame (header null = the raw header named a reserved
   * channel but failed validation). Only a well-formed @control hello is
   * legal before registration; violations reject the session (error
   * datagram + close). After registration, unknown control is dropped - a
   * registered peer's bytes must not kill its live session. */
  #onControlFrame(header: FrameHeader | null, bytes: Uint8Array): void {
    // Control payloads reuse the datagram encoding; bytes are a complete
    // frame, so the lengths peek cannot fail.
    const lens = peekDataFrameLengths(bytes)!;
    const msg = header === null ? null : decodeDatagram(bytes.subarray(8 + lens.headerLen));
    if (header === null || header.ch !== CONTROL_CHANNEL || msg === null || msg.t !== "hello") {
      if (this.info === null) {
        this.#reject(
          "invalid_control",
          "only a hello @control frame is accepted before registration",
          "invalid control frame",
        );
      } else {
        console.log(
          `[relay] dropping unknown robot control frame (ch ${header?.ch ?? "invalid header"})`,
        );
      }
      return;
    }
    this.#onHello(msg);
  }

  /** Hello validation, identical to the v4 datagram path, plus the v5
   * mutation guard: a resend (healing a lost welcome datagram) must be
   * byte-identical in effect - identity or manifest changes are rejected. */
  #onHello(msg: HelloMsg): void {
    if (msg.v !== PROTOCOL_VERSION) {
      this.#reject(
        "version_mismatch",
        `protocol v${PROTOCOL_VERSION} required, got v${msg.v}`,
        "version mismatch",
      );
      return;
    }
    if (msg.role !== "robot") {
      this.#reject("role_mismatch", "the /robot endpoint requires role=robot", "role mismatch");
      return;
    }
    if (msg.robot === undefined) {
      this.#reject(
        "missing_robot_id",
        "robot hello must carry robot{id,name,model}",
        "missing robot id",
      );
      return;
    }
    // Manifest-less hellos are legal (transport tests); a declared manifest
    // must pass the domain rules (incl. the version gate) or duplicate/
    // bogus channels would be interpreted inconsistently downstream.
    let channels: ChannelSpec[] = [];
    if (msg.manifest !== undefined) {
      try {
        channels = parseManifest(msg.manifest).channels;
      } catch (e) {
        this.#reject("invalid_manifest", (e as Error).message, "invalid manifest");
        return;
      }
    }
    if (this.info === null) {
      this.info = msg.robot;
      this.channels = channels;
      this.manifest = msg.manifest ?? null;
    } else if (
      this.info.id !== msg.robot.id ||
      this.info.name !== msg.robot.name ||
      this.info.model !== msg.robot.model ||
      JSON.stringify(this.manifest) !== JSON.stringify(msg.manifest ?? null)
    ) {
      this.#reject(
        "hello_mismatch",
        "hello may not change robot identity or manifest on a live session",
        "hello mismatch",
      );
      return;
    }
    if (!this.#registry.registerRobot(this)) {
      this.#reject(
        "robot_id_conflict",
        `robot id ${msg.robot.id} already has a live session`,
        "robot id conflict",
      );
      return;
    }
    this.sendMsg({ t: "welcome", v: PROTOCOL_VERSION });
  }

  #frameLoop(): void {
    (async () => {
      for await (const bidi of this.#conn.incomingBidirectionalStreams) {
        bidi.writable.abort().catch(() => {});
        (async () => {
          await readWebTransportPreamble(bidi.readable);
          const { header, bytes, reserved } = await readRobotFrame(bidi.readable);
          if (reserved) {
            this.#onControlFrame(header, bytes);
          } else {
            this.#registry.onRobotFrame(this, bytes, header);
          }
        })().catch((e) => {
          if (e instanceof ControlPayloadTooLargeError) {
            this.#reject("control_too_large", e.message, "oversized control frame");
            return;
          }
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
      sendFrame(bytes: Uint8Array): FrameSend {
        let aborted = false;
        let writeStarted = false;
        let writer: WritableStreamDefaultWriter<Uint8Array> | null = null;
        let settle!: () => void;
        let fail!: (e: unknown) => void;
        const done = new Promise<void>((resolve, reject) => {
          settle = resolve;
          fail = reject;
        });
        const reset = () => {
          writer?.abort(
            new WebTransportError("stale frame superseded", {
              source: "stream",
              streamErrorCode: STALE_STREAM_ERROR_CODE,
            }),
          ).catch(() => {});
        };
        (async () => {
          const stream = await wt.createUnidirectionalStream({
            waitUntilAvailable: true,
            sendOrder: -(latestOrder++),
          });
          // Keep the writer for the stream's whole life: aborts go through it
          // (the stream itself stays locked).
          writer = stream.getWriter();
          if (aborted) {
            // abort() raced the create (done already rejected); release the
            // just-granted stream credit.
            reset();
            return;
          }
          // Latching writeStarted and starting the write in one synchronous
          // step is what makes supersede race-free: the payload can no
          // longer change once bytes may have reached the transport.
          writeStarted = true;
          await writer.write(bytes);
          settle();
          // No close(): a closed stream cannot be aborted, and every latest
          // stream ends in a reset (reap, dispose, or session teardown).
          // Receivers dispatch on byte count and treat the reset as EOF.
        })().catch((e) => fail(e)); // no-op if done already settled
        return {
          done,
          get aborted() {
            return aborted;
          },
          abort() {
            if (aborted) return;
            aborted = true;
            fail(new Error("frame send aborted"));
            reset();
          },
          supersede(newBytes: Uint8Array): boolean {
            if (writeStarted || aborted) return false;
            bytes = newBytes; // the write reads `bytes` only after create resolves
            return true;
          },
        };
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
