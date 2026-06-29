// WebCodecsMedia — the browser hardware-decodes the camera itself. The gateway H.264-encodes the
// camera Image (PyAV libx264, Annex-B) and streams raw NAL chunks over its WebSocket; here a
// WebCodecs VideoDecoder turns them into VideoFrames the canvas blits (onFrame). Same codec as
// WebRTC, but NO ICE/SDP — it rides the WS (so it works behind any data transport), gives the app
// frame-level access, and the gateway encodes ONCE for N viewers. selectMediaChannel falls back to
// the jpeg floor when VideoDecoder is unavailable.
//
// Wire (gateway → browser): a JSON {op:"video-config", topic, codec} when a sub starts, then binary
//   [u8 flags(bit0=keyframe)][u64 ts_us BE][u16 topic_len BE][topic utf8][H.264 Annex-B NAL]
import type { MediaCaps, MediaChannel, VideoMeta } from "../media";
import type { Status } from "../transport";

interface Decoding {
  decoder: VideoDecoder;
  sawKey: boolean; // a decoder can only start on a keyframe
  meta: VideoMeta;
}

export class WebCodecsMedia implements MediaChannel {
  readonly caps: MediaCaps = {
    output: "frames",
    codec: "h264",
    onDemand: true,
    multiStream: true,
    hardwareDecode: true,
  };
  label = "webcodecs (h264)";
  private ws?: WebSocket;
  private decoders = new Map<string, Decoding>();
  private wanted = new Set<string>(); // (re)start these once the ws opens / reopens
  private frameCb?: (id: string, frame: VideoFrame | ImageBitmap, m: VideoMeta) => void;
  private statusCb?: (s: Status) => void;

  constructor(private gatewayUrl: string) {}

  connect(): Promise<void> {
    if (this.ws && this.ws.readyState <= WebSocket.OPEN) return Promise.resolve();
    return new Promise((resolve, reject) => {
      this.statusCb?.("connecting");
      const ws = new WebSocket(this.gatewayUrl);
      ws.binaryType = "arraybuffer";
      this.ws = ws;
      ws.onopen = () => {
        this.statusCb?.("open");
        for (const t of this.wanted) this.send({ op: "webcodecs-start", topic: t });
        resolve();
      };
      ws.onmessage = (e) => this.onMessage(e);
      ws.onerror = () => reject(new Error("webcodecs ws error"));
      ws.onclose = () => this.statusCb?.("closed");
    });
  }

  private onMessage(e: MessageEvent): void {
    if (typeof e.data === "string") {
      let m: { op?: string; topic?: string; codec?: string };
      try {
        m = JSON.parse(e.data);
      } catch {
        return;
      }
      if (m.op === "video-config" && m.topic) this.configure(m.topic, m.codec);
      return; // hello / others: ignore (no data-plane on this socket)
    }
    const buf = e.data as ArrayBuffer;
    if (buf.byteLength < 11) return;
    const dv = new DataView(buf);
    const flags = dv.getUint8(0);
    const tsUs = Number(dv.getBigUint64(1));
    const topicLen = dv.getUint16(9);
    const topic = new TextDecoder().decode(new Uint8Array(buf, 11, topicLen));
    const payload = new Uint8Array(buf, 11 + topicLen);
    this.decodeChunk(topic, payload, (flags & 1) === 1, tsUs);
  }

  private configure(topic: string, codec?: string): void {
    const c = codec || "avc1.42E01F";
    const prev = this.decoders.get(topic);
    if (prev) {
      try {
        prev.decoder.close();
      } catch {
        /* already closed */
      }
    }
    const meta: VideoMeta = { width: 0, height: 0, fps: 0, codec: c };
    const decoder = new VideoDecoder({
      output: (frame: VideoFrame) => {
        meta.width = frame.displayWidth || frame.codedWidth;
        meta.height = frame.displayHeight || frame.codedHeight;
        this.frameCb?.(topic, frame, meta); // caller draws then closes the frame
      },
      error: () => {
        const d = this.decoders.get(topic);
        if (d) d.sawKey = false; // resync on the next keyframe
      },
    });
    // Don't force hardwareAcceleration:"prefer-hardware" — in some Chrome contexts configure()
    // succeeds but the decoder then errors asynchronously (no frames ever output). Letting the
    // browser choose (sw/hw) is what actually decodes; it still uses hardware when available.
    try {
      decoder.configure({ codec: c, optimizeForLatency: true });
    } catch {
      try {
        decoder.configure({ codec: c }); // codec alone is enough for Annex-B in-band SPS/PPS
      } catch {
        return;
      }
    }
    this.decoders.set(topic, { decoder, sawKey: false, meta });
  }

  private decodeChunk(topic: string, data: Uint8Array, isKey: boolean, tsUs: number): void {
    const d = this.decoders.get(topic);
    if (!d) return; // config (→ decoder) always precedes chunks
    if (!d.sawKey && !isKey) return; // can't start mid-GOP
    d.sawKey = true;
    try {
      d.decoder.decode(new EncodedVideoChunk({ type: isKey ? "key" : "delta", timestamp: tsUs, data }));
    } catch {
      d.sawKey = false; // wait for the next keyframe and resync
    }
  }

  subscribe(streamId: string): void {
    this.wanted.add(streamId);
    if (this.ws?.readyState === WebSocket.OPEN) this.send({ op: "webcodecs-start", topic: streamId });
  }

  unsubscribe(streamId: string): void {
    this.wanted.delete(streamId);
    this.send({ op: "webcodecs-stop", topic: streamId });
    const d = this.decoders.get(streamId);
    if (d) {
      try {
        d.decoder.close();
      } catch {
        /* already closed */
      }
      this.decoders.delete(streamId);
    }
  }

  onStream(): void {} // n/a — this channel is "frames"
  onFrame(cb: (id: string, frame: VideoFrame | ImageBitmap, m: VideoMeta) => void): void {
    this.frameCb = cb;
  }
  onStatus(cb: (s: Status) => void): void {
    this.statusCb = cb;
  }

  private send(obj: unknown): void {
    if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj));
  }

  close(): void {
    for (const d of this.decoders.values()) {
      try {
        d.decoder.close();
      } catch {
        /* already closed */
      }
    }
    this.decoders.clear();
    this.wanted.clear();
    this.ws?.close();
  }
}
