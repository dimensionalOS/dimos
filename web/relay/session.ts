// Per-connection session objects: handshake, control loops, and (robot leg)
// the raw-QUIC data stream loop. Sessions own transport quirks; all routing
// and subscription policy lives in registry.ts.
//
// Leg asymmetry, forced by upstream bugs (see docs/web/protocol.md):
// - Robot (aioquic): robot->relay hello rides an @control data frame on a
//   one-shot bidi stream (v5); relay->robot handshake and teleop control
//   (welcome, errors, pong, teleop) rides datagrams, while subs snapshots
//   (and future robot-bound control) ride @control frames on the reliable
//   robot control carrier - ONE relay-opened uni stream per session. The
//   relay still never writes on robot-OPENED bidi streams (their send half
//   is aborted with RESET, never FIN); the carrier is relay-opened, the
//   proven direction. Data = one-shot bidi streams under the same rule.
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
import type { Auth } from "./auth.ts";
import { parseManifest, TRACK_ENCODING } from "@dimos/shared/manifest";
import { type CarrierStats, RobotCarrier } from "./carrier.ts";
import {
  type ChannelPolicy,
  CONTROL_SEND_ORDER,
  ControlPayloadTooLargeError,
  type FrameWriter,
  IncompleteFrameError,
  readRobotFrames,
  type ViewerSink,
} from "./forward.ts";
import type { Registry, RobotPeer, ViewerPeer } from "./registry.ts";

export { CONTROL_SEND_ORDER, LATEST_SEND_ORDER, RELIABLE_SEND_ORDER } from "./forward.ts";

// A viewer stream create pending this long has run out of the viewer's
// incoming-stream credit (~100 on every client). Disposed streams (an
// unsubscribed channel's) return their credit only when Deno finalizes
// their wrappers, so the stall forces a collection (forceGc) and is counted
// in /api/stats; a create still pending at the kick deadline means the
// viewer holds every slot and is disconnected.
export const STREAM_CREDIT_STALL_MS = 1000;
export const STREAM_CREDIT_KICK_MS = 10_000;
// Idle streams kept per viewer and sendOrder for reuse (sink.releaseStream);
// beyond this an unsubscribed channel's stream is abandoned as before.
const IDLE_STREAMS_MAX = 64;

// How long a robot session holding a contested id has to answer the
// relay's ping before a new session with that id supersedes it (the bridge
// answers at once; a killed one never does).
export const ROBOT_PROBE_MS = 1000;

/**
 * Force a V8 garbage collection and record its pause. Deno 2.6.10 ends a
 * stream (FIN or reset) only when its wrapper is finalized, so collection is
 * what returns a finished robot stream's slot in the relay's incoming-stream
 * budget and a disposed viewer stream's credit on the viewer. gc() exists
 * only under --v8-flags=--expose-gc (relay_run_cmd passes it).
 */
export function forceGc(registry: Registry): void {
  const gc = (globalThis as { gc?: () => void }).gc;
  if (gc === undefined) return;
  const started = performance.now();
  gc();
  registry.collected(performance.now() - started);
}

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
  readonly #carrier: RobotCarrier;
  readonly #auth: Auth | null;
  /** The registration claim in progress (hello resends wait on it). */
  #claim: Promise<void> | null = null;
  #probeN = 0;
  readonly #probeWaiters = new Map<number, (alive: boolean) => void>();
  readonly #probeMs: number;

  constructor(
    wt: WebTransport,
    conn: Deno.QuicConn,
    registry: Registry,
    auth: Auth | null = null,
    timing: { probeMs?: number } = {},
  ) {
    this.#wt = wt;
    this.#conn = conn;
    this.#registry = registry;
    this.#auth = auth;
    this.#probeMs = timing.probeMs ?? ROBOT_PROBE_MS;
    this.#dgWriter = wt.datagrams.writable.getWriter();
    // The stream opens lazily on the first sendControl - the registration
    // baseline snapshot - so a rejected session never burns a stream.
    this.#carrier = new RobotCarrier({
      openStream: async () => {
        const stream = await wt.createUnidirectionalStream({
          waitUntilAvailable: true,
          sendOrder: CONTROL_SEND_ORDER,
        });
        return stream.getWriter();
      },
      fail: (reason) => this.#failCarrier(reason),
    });
  }

  sendMsg(msg: Msg): void {
    this.#dgWriter.write(encodeDatagram(msg)).catch(() => {});
  }

  sendControl(msg: Msg): void {
    this.#carrier.sendControl(msg);
  }

  sendPub(ch: string, payload: Uint8Array, meta: Record<string, unknown>): void {
    this.#carrier.sendFrame(ch, payload, meta);
  }

  carrierStats(): CarrierStats {
    return this.#carrier.stats();
  }

  probe(): Promise<boolean> {
    if (this.closed !== null) return Promise.resolve(false);
    const n = ++this.#probeN;
    return new Promise((resolve) => {
      const timer = setTimeout(() => {
        this.#probeWaiters.delete(n);
        resolve(false);
      }, this.#probeMs);
      this.#probeWaiters.set(n, () => {
        clearTimeout(timer);
        resolve(true);
      });
      this.sendMsg({ t: "ping", n, ts: Date.now() / 1000 });
    });
  }

  supersede(): void {
    this.#reject("robot_superseded", "a new session registered this robot id", "superseded");
  }

  /** The carrier is a control dependency: on overflow or write failure the
   * whole session fails (error datagram + close), never limps on with stale
   * subscription state. The bridge reconnects and gets a fresh baseline. */
  #failCarrier(reason: string): void {
    if (this.closed !== null) return; // already tearing down: not a failure
    this.#registry.carrierFailed();
    console.error(`[relay] robot control carrier failed: ${reason}`);
    this.#reject("carrier_failed", `robot control carrier failed: ${reason}`, "carrier failure");
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
      .finally(() => {
        this.#carrier.dispose();
        this.#registry.robotClosed(this);
      });
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
    if (msg.t === "pong") {
      const waiter = this.#probeWaiters.get(msg.n);
      if (waiter !== undefined) {
        this.#probeWaiters.delete(msg.n);
        waiter(true);
      }
    }
    return true;
  }

  /** One @-channel frame (header null = the raw header named a reserved
   * channel but failed validation). Only a well-formed @control hello is
   * legal before registration; violations reject the session (error
   * datagram + close). After registration, publish acks route to the
   * registry and other unknown control is dropped - a registered peer's
   * bytes must not kill its live session. */
  #onControlFrame(header: FrameHeader | null, bytes: Uint8Array): void {
    // Control payloads reuse the datagram encoding; bytes are a complete
    // frame, so the lengths peek cannot fail.
    const lens = peekDataFrameLengths(bytes)!;
    const msg = header === null ? null : decodeDatagram(bytes.subarray(8 + lens.headerLen));
    if (header !== null && header.ch === CONTROL_CHANNEL && msg !== null) {
      if (msg.t === "hello") {
        this.#onHello(msg);
        return;
      }
      if (this.info !== null && (msg.t === "pub_ack" || msg.t === "pub_nack")) {
        this.#registry.onRobotPubResult(this, msg);
        return;
      }
      if (this.info !== null && msg.t === "rtc_offer") {
        this.#registry.onRobotRtcOffer(this, msg);
        return;
      }
      if (this.info !== null && msg.t === "rtc_stalled") {
        this.#registry.onRobotRtcStalled(this, msg);
        return;
      }
    }
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
    if (this.#auth !== null && !this.#auth.robotKeyOk(msg.robot.id, msg.token)) {
      const message = msg.token === undefined ? "missing robot key" : "invalid robot key";
      // Resends keep arriving until the close lands; log the first only.
      if (this.closed === null) console.log(`[relay] robot ${msg.robot.id} rejected: ${message}`);
      this.#reject("auth_failed", message, "auth failed");
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
      if (!this.#registry.rtcEnabled && channels.some((c) => c.encoding === TRACK_ENCODING)) {
        // The bridge only declares track channels when /api/info advertised
        // rtc, so this is a relay reconfigured under a live robot.
        this.#reject(
          "rtc_unavailable",
          `${TRACK_ENCODING} channels need a relay with Cloudflare configured (--rtc-file)`,
          "rtc unavailable",
        );
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
    // The claim may probe a previous holder of the id for up to ROBOT_PROBE_MS;
    // hello resends meanwhile wait for its outcome.
    if (this.#claim !== null) return;
    const robotId = msg.robot.id;
    this.#claim = this.#registry.claimRobot(this).then((registered) => {
      this.#claim = null;
      if (this.closed !== null) return;
      if (!registered) {
        this.#reject(
          "robot_id_conflict",
          `robot id ${robotId} already has a live session`,
          "robot id conflict",
        );
        return;
      }
      this.sendMsg({ t: "welcome", v: PROTOCOL_VERSION });
    }).catch((e) => {
      this.#claim = null;
      console.log(`[relay] robot ${robotId} claim failed:`, (e as Error)?.message ?? e);
    });
  }

  /**
   * Robot data streams: one frame per latest or @control stream, a reliable
   * channel's frames back to back on its persistent stream (v8), so they
   * arrive in order with no stream per frame. Each stream is read in its own
   * task and every frame routes as soon as it is complete.
   */
  #frameLoop(): void {
    (async () => {
      for await (const bidi of this.#conn.incomingBidirectionalStreams) {
        bidi.writable.abort().catch(() => {});
        readRobotFrames(bidi.readable, ({ header, bytes, reserved }) => {
          if (reserved) this.#onControlFrame(header, bytes);
          else this.#registry.onRobotFrame(this, bytes, header);
        }).catch((e) => {
          if (e instanceof ControlPayloadTooLargeError) {
            this.#reject("control_too_large", e.message, "oversized control frame");
            return;
          }
          // Reset mid-frame (the bridge reset a stale latest frame, or the
          // connection died): the partial is dropped.
          if (e instanceof IncompleteFrameError) this.#registry.robotStreamIncomplete();
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
  /** Viewer name from the auth file (auth on), for the log and /api/stats. */
  name: string | null = null;
  readonly sink: ViewerSink;
  readonly #wt: WebTransport;
  readonly #registry: Registry;
  readonly #auth: Auth | null;
  /** Push channel for robots events, chosen by whichever leg carried hello. */
  #push: ((msg: Msg) => void) | null = null;
  readonly #creditStallMs: number;
  readonly #creditKickMs: number;

  constructor(
    wt: WebTransport,
    id: number,
    registry: Registry,
    auth: Auth | null = null,
    timing: { creditStallMs?: number; creditKickMs?: number } = {},
  ) {
    this.#wt = wt;
    this.id = id;
    this.#registry = registry;
    this.#auth = auth;
    this.#creditStallMs = timing.creditStallMs ?? STREAM_CREDIT_STALL_MS;
    this.#creditKickMs = timing.creditKickMs ?? STREAM_CREDIT_KICK_MS;
    this.sink = {
      openStream: (order) => {
        const idle = this.#idle.get(order)?.pop();
        return idle === undefined ? this.#openStream(order) : Promise.resolve(idle);
      },
      sharedStream: (order) => this.#shared ??= this.#openStream(order),
      releaseStream: (order, writer) => {
        const idle = this.#idle.get(order) ?? [];
        if (idle.length >= IDLE_STREAMS_MAX) {
          writer.abort().catch(() => {});
          return;
        }
        idle.push(writer);
        this.#idle.set(order, idle);
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

  /** The stream the viewer's small reliable channels share (sink.sharedStream). */
  #shared: Promise<FrameWriter> | null = null;
  /** Streams of unsubscribed channels, by sendOrder, for the next channel at
   * that order (sink.releaseStream): a page switch would otherwise abandon a
   * stream per panel, which on Deno 2.6.10 lives on for the connection. */
  #idle = new Map<number, FrameWriter[]>();

  /** One persistent uni stream per channel at its band's sendOrder. The
   * create waits for stream credit (protocol.md workaround 7); a long wait
   * is counted, forces a GC, and past the kick deadline disconnects the
   * viewer (see STREAM_CREDIT_STALL_MS). */
  async #openStream(order: number): Promise<FrameWriter> {
    const create = this.#wt.createUnidirectionalStream({
      waitUntilAvailable: true,
      sendOrder: order,
    });
    const stall = setTimeout(() => {
      this.#registry.streamCreditStalled();
      forceGc(this.#registry);
    }, this.#creditStallMs);
    const kick = setTimeout(() => this.sink.kick("stream credit exhausted"), this.#creditKickMs);
    try {
      return (await create).getWriter();
    } finally {
      clearTimeout(stall);
      clearTimeout(kick);
    }
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
    const authorized = msg.t !== "hello" || this.#authorize(msg, reply);
    if (!authorized || !this.#registry.onViewerMsg(this, msg, reply)) {
      closeAfterFlush(this.#wt, "viewer handshake rejected");
      return false;
    }
    if (msg.t === "hello" && this.greeted) this.#push = reply;
    return true;
  }

  /** With auth on, a hello must carry a known viewer token; the name sticks
   * to the session. Version and role mismatches are left to the registry so
   * their codes keep precedence over auth_failed. */
  #authorize(msg: HelloMsg, reply: (msg: Msg) => void): boolean {
    if (this.#auth === null || msg.v !== PROTOCOL_VERSION || msg.role !== "viewer") return true;
    const name = this.#auth.viewerName(msg.token);
    if (name === null) {
      const message = msg.token === undefined ? "missing viewer token" : "invalid viewer token";
      console.log(`[relay] viewer ${this.id} rejected: ${message}`);
      reply({ t: "error", code: "auth_failed", message });
      return false;
    }
    if (this.name === null) console.log(`[relay] viewer ${this.id} authenticated as ${name}`);
    this.name = name;
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
