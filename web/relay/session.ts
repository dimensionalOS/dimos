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
  DataFrameStreamReader,
  decodeDatagram,
  encodeControlFrame,
  encodeDatagram,
  type FrameHeader,
  type HelloMsg,
  MAX_TOPIC_GUARD_BYTES,
  MAX_TOPIC_PAYLOAD_BYTES,
  type Msg,
  peekDataFrameLengths,
  PROTOCOL_VERSION,
  type RobotInfo,
  type RobotManifest,
  TOPIC_GUARD_CHANNEL,
  TOPIC_GUARD_STREAM_MAGIC,
  TOPIC_PUBLISH_CHANNEL,
  TOPIC_PUBLISH_STREAM_MAGIC,
  type TopicMsg,
  topicRouteOk,
} from "@dimos/shared";
import type { Auth } from "./auth.ts";
import { parseManifest, TRACK_ENCODING } from "@dimos/shared/manifest";
import { type CarrierStats, RobotCarrier } from "./carrier.ts";
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

const MAX_PROVIDER_INGRESS_BYTES = 64 * 1024 * 1024;
class IngressOverflow extends Error {}

let resetStreamsPendingCollection = false;
/** Deno still needs wrapper collection to return aborted stream credit.
 * 2.9.8 removes the per-stream connection-close callbacks retained by 2.6. */
export function collectResetStreams(): void {
  if (!resetStreamsPendingCollection) return;
  resetStreamsPendingCollection = false;
  (globalThis as { gc?: () => void }).gc?.();
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
  topicSource: string | null = null;
  #streams = 0;
  #ingressBytes = 0;
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
  #topicWriter: Promise<FrameWriter> | null = null;
  #topicWrites: Promise<void> = Promise.resolve();
  #guardWriter: Promise<FrameWriter> | null = null;
  #guardWrites: Promise<void> = Promise.resolve();

  constructor(
    wt: WebTransport,
    conn: Deno.QuicConn,
    registry: Registry,
    auth: Auth | null = null,
  ) {
    this.#wt = wt;
    this.#conn = conn;
    this.#registry = registry;
    this.#auth = auth;
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

  sendTopicPublication(bytes: Uint8Array): void {
    if (this.closed !== null) return;
    // Registry receipts bound this chain's queued + transport-buffered bytes.
    // A second persistent stream keeps bulk writes behind control priority.
    this.#topicWrites = this.#topicWrites.then(async () => {
      if (this.closed !== null) return;
      this.#topicWriter ??= this.#wt.createUnidirectionalStream({
        waitUntilAvailable: true,
        sendOrder: 0,
      }).then((stream) => stream.getWriter());
      const writer = await this.#topicWriter;
      if (this.closed !== null) {
        await writer.abort();
        return;
      }
      await writer.write(bytes);
    }).catch(() => this.failTopics("topic_write_failed"));
  }

  sendTopicGuard(bytes: Uint8Array): void {
    if (this.closed !== null) return;
    // The registry caps eight unacknowledged 8 KiB commands per handle.
    // Keep this lane ahead of bulk writes and behind stop/heartbeat control.
    this.#guardWrites = this.#guardWrites.then(async () => {
      if (this.closed !== null) return;
      this.#guardWriter ??= this.#wt.createUnidirectionalStream({
        waitUntilAvailable: true,
        sendOrder: CONTROL_SEND_ORDER - 1,
      }).then((stream) => stream.getWriter());
      const writer = await this.#guardWriter;
      if (this.closed !== null) {
        await writer.abort();
        return;
      }
      await writer.write(bytes);
    }).catch(() => this.failTopics("guard_write_failed"));
  }

  failTopics(reason: string): void {
    this.#reject("topic_source_failed", reason, reason);
  }

  carrierStats(): CarrierStats {
    return this.#carrier.stats();
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
        this.closed ??= "connection closed";
        this.#topicWriter?.then((writer) => writer.abort()).catch(() => {});
        this.#guardWriter?.then((writer) => writer.abort()).catch(() => {});
        this.#carrier.dispose();
        this.#registry.topics.sourceClosed(this);
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
      if (this.topicSource !== null && msg.t.startsWith("topic_")) {
        this.#registry.topics.onProviderMsg(this, msg as TopicMsg);
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
    if (this.info === null && this.topicSource === null) {
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
    if (msg.topicSource !== undefined || this.topicSource !== null) {
      if (
        msg.robot !== undefined || msg.manifest !== undefined || this.info !== null ||
        msg.topicsVersion !== 1 || msg.topicSource === undefined
      ) {
        this.#reject(
          "invalid_topic_hello",
          "topic source requires a standalone v1 provider",
          "topic hello",
        );
        return;
      }
      if (!this.#registry.topics.open) {
        this.#reject(
          "topics_disabled",
          "raw topics need a relay started with --open-topics",
          "topics disabled",
        );
        return;
      }
      if (this.#auth !== null && !this.#auth.topicSourceKeyOk(msg.topicSource, msg.token)) {
        this.#reject("auth_failed", "invalid topic provider key", "topic auth");
        return;
      }
      if (this.closed !== null) return;
      if (this.topicSource !== null && this.topicSource !== msg.topicSource) {
        this.#reject("hello_mismatch", "topic source cannot change", "topic source changed");
        return;
      }
      const refusal = this.#registry.topics.registerSource(
        this,
        msg.topicSource,
        msg.topicCapabilities,
      );
      if (refusal !== null) {
        this.#reject(
          refusal,
          refusal === "topic_source_limit"
            ? "the relay already serves as many topic sources as it can list"
            : "topic source already connected",
          "topic source refused",
        );
        return;
      }
      this.topicSource = msg.topicSource;
      this.sendMsg({ t: "welcome", v: PROTOCOL_VERSION });
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
    // Manifest-less hellos grant no subscriptions; a declared manifest
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
        if (this.#streams >= (this.info !== null || this.topicSource !== null ? 256 : 8)) {
          bidi.readable.cancel().catch(() => {});
          continue;
        }
        this.#streams++;
        const stream = Symbol("provider stream");
        let retained = 0;
        const retain = (bytes: number) => {
          if (this.#ingressBytes + bytes > MAX_PROVIDER_INGRESS_BYTES) {
            throw new IngressOverflow("provider partial frames exceed aggregate byte bound");
          }
          this.#ingressBytes += bytes;
          retained += bytes;
        };
        const release = () => {
          this.#ingressBytes -= retained;
          retained = 0;
        };
        (async () => {
          await readWebTransportPreamble(bidi.readable);
          const { header, bytes, reserved } = await readRobotFrame(
            bidi.readable,
            this.topicSource !== null,
            retain,
          );
          if (this.topicSource !== null && header !== null && topicRouteOk(header.ch)) {
            const route = header.ch;
            const reliable = header.delivery === "reliable";
            const lens = peekDataFrameLengths(bytes)!;
            this.#registry.topics.onFrame(this, header, bytes.subarray(8 + lens.headerLen), stream);
            release();
            if (!reliable) return;
            // Persistent routes amortize QUIC reads over a chunk. Reading
            // prefix/header/payload separately for every tiny event caused
            // thousands of async native calls in the event baseline.
            const reader = bidi.readable.getReader({ mode: "byob" });
            const frames = new DataFrameStreamReader(MAX_TOPIC_PAYLOAD_BYTES);
            try {
              while (true) {
                const { value, done } = await reader.read(new Uint8Array(64 * 1024));
                if (done) break;
                retain(value.byteLength);
                const decoded = frames.push(value);
                release();
                retain(frames.bufferedBytes);
                for (const frame of decoded) {
                  if (frame.header.ch !== route || frame.header.delivery !== "reliable") return;
                  this.#registry.topics.onFrame(this, frame.header, frame.payload, stream);
                }
              }
            } finally {
              reader.releaseLock();
            }
            return;
          }
          if (reserved) {
            this.#onControlFrame(header, bytes);
          } else {
            this.#registry.onRobotFrame(this, bytes, header);
          }
        })().catch((e) => {
          if (e instanceof IngressOverflow) {
            this.#reject("topic_ingress_overflow", e.message, "provider ingress overflow");
            return;
          }
          if (e instanceof ControlPayloadTooLargeError) {
            this.#reject("control_too_large", e.message, "oversized control frame");
            return;
          }
          // reset before/mid-frame (stale latest-wins write): drop the partial
        }).finally(() => {
          release();
          this.#streams--;
          this.#registry.topics.streamClosed(this, stream);
          bidi.readable.cancel().catch(() => {});
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
  #bulkStarted = false;
  #guardStarted = false;
  #streams = 0;
  #ended = false;

  constructor(wt: WebTransport, id: number, registry: Registry, auth: Auth | null = null) {
    this.#wt = wt;
    this.id = id;
    this.#registry = registry;
    this.#auth = auth;
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
          resetStreamsPendingCollection = true;
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
      kick: (reason: string): void => {
        // Transport shutdown may wait for a blocked/reset stream to drain.
        // Authorization and provider demand end immediately, not when that
        // asynchronous WebTransport.closed promise eventually settles.
        this.#end();
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
    if (this.#ended) return;
    this.#push?.(msg);
  }

  #end(): void {
    if (this.#ended) return;
    this.#ended = true;
    this.#registry.viewerClosed(this);
    this.#push = null;
    console.log(`[relay] viewer ${this.id} disconnected`);
  }

  start(): void {
    this.#registry.addViewer(this);
    console.log(`[relay] viewer ${this.id} connected`);
    this.#wt.closed
      .catch(() => {})
      .finally(() => this.#end());
    this.#streamLoop();
    this.#datagramLoop();
  }

  #dispatch(msg: Msg, reply: (msg: Msg) => void): boolean {
    if (this.#ended) return false;
    const authorized = msg.t !== "hello" || this.#authorize(msg, reply);
    if (!authorized || !this.#registry.onViewerMsg(this, msg, reply)) {
      this.#end();
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
        // A browser needs one control stream and one binary publication
        // stream. Bound incomplete control frames from authenticated peers too.
        if (this.#streams >= 8) {
          bidi.readable.cancel().catch(() => {});
          bidi.writable.abort().catch(() => {});
          this.sink.kick("viewer_stream_limit");
          return;
        }
        this.#streams++;
        (async () => {
          // Viewer-opened, so its send half starts at the default order 0,
          // which a saturated reliable stream would starve.
          bidi.writable.sendOrder = CONTROL_SEND_ORDER;
          const writer = bidi.writable.getWriter();
          const reply = (m: Msg) => {
            writer.write(encodeControlFrame(m)).catch(() => {});
          };
          const frames = new ControlFrameReader();
          const prefix = new Uint8Array(4);
          let prefixBytes = 0;
          let bulk: DataFrameStreamReader | null = null;
          let guarded = false;
          try {
            for await (let chunk of bidi.readable) {
              if (prefixBytes < 4) {
                const n = Math.min(4 - prefixBytes, chunk.byteLength);
                prefix.set(chunk.subarray(0, n), prefixBytes);
                prefixBytes += n;
                chunk = chunk.subarray(n);
                if (prefixBytes < 4) continue;
                const magic = new DataView(prefix.buffer).getUint32(0, true);
                if (magic === TOPIC_PUBLISH_STREAM_MAGIC || magic === TOPIC_GUARD_STREAM_MAGIC) {
                  guarded = magic === TOPIC_GUARD_STREAM_MAGIC;
                  if (
                    !this.greeted || !this.#registry.topics.open ||
                    (guarded ? this.#guardStarted : this.#bulkStarted)
                  ) {
                    this.sink.kick("topic_write_denied");
                    return;
                  }
                  if (guarded) this.#guardStarted = true;
                  else this.#bulkStarted = true;
                  bulk = new DataFrameStreamReader(
                    guarded ? MAX_TOPIC_GUARD_BYTES : MAX_TOPIC_PAYLOAD_BYTES,
                  );
                  writer.abort().catch(() => {});
                } else frames.push(prefix);
              }
              if (bulk !== null) {
                for (const frame of bulk.push(chunk)) {
                  if (frame.header.ch !== (guarded ? TOPIC_GUARD_CHANNEL : TOPIC_PUBLISH_CHANNEL)) {
                    this.sink.kick("invalid_publication_lane");
                    return;
                  }
                  this.#registry.topics.publish(this, frame.header, frame.payload);
                }
              } else {
                for (const msg of frames.push(chunk)) {
                  if (!this.#dispatch(msg, reply)) return;
                }
              }
            }
          } finally {
            writer.releaseLock();
            if (bulk !== null) this.sink.kick("topic_publish_stream_closed");
            else if (this.#push === reply) this.sink.kick("viewer_control_stream_closed");
          }
        })().catch((e) =>
          console.log("[relay] viewer control stream ended:", (e as Error)?.message ?? e)
        ).finally(() => {
          this.#streams--;
        });
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
