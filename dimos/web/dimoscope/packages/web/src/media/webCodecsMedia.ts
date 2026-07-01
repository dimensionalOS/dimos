// gateway H.264-encodes the camera (PyAV libx264, Annex-B), streams raw NAL chunks over its
// WebSocket; browser hardware-decodes via WebCodecs; no ICE/SDP; falls back to jpeg when
// VideoDecoder unavailable.
//
// Wire (gateway → browser): a JSON {op:"video-config", topic, codec} when a sub starts, then binary
//   [u8 flags(bit0=keyframe)][u64 ts_us BE][u16 topic_len BE][topic utf8][H.264 Annex-B NAL]
import type { MediaCaps, MediaChannel, VideoMeta } from "../media.ts";
import type { Status } from "../transport.ts";

interface Decoding {
  decoder: VideoDecoder;
  sawKey: boolean; // a decoder can only start on a keyframe
  meta: VideoMeta;
}

export interface WebCodecsMediaDeps {
  gatewayUrl: string; // media node WS that streams H.264 chunks (e.g. ws://host:8092)
}

export const createWebCodecsMedia = (deps: WebCodecsMediaDeps): MediaChannel => {
  const { gatewayUrl } = deps;
  const caps: MediaCaps = {
    output: "frames",
    codec: "h264",
    onDemand: true,
    multiStream: true,
    hardwareDecode: true,
  };
  let ws: WebSocket | undefined;
  const decoders = new Map<string, Decoding>();
  const wanted = new Set<string>(); // (re)start these once the ws opens / reopens
  let frameCb: ((id: string, frame: VideoFrame | ImageBitmap, m: VideoMeta) => void) | undefined;
  let statusCb: ((s: Status) => void) | undefined;

  function connect(): Promise<void> {
    if (ws && ws.readyState <= WebSocket.OPEN) return Promise.resolve();
    return new Promise((resolve, reject) => {
      statusCb?.("connecting");
      const sock = new WebSocket(gatewayUrl);
      sock.binaryType = "arraybuffer";
      ws = sock;
      sock.onopen = () => {
        statusCb?.("open");
        for (const t of wanted) send({ op: "webcodecs-start", topic: t });
        resolve();
      };
      sock.onmessage = (e) => onMessage(e);
      sock.onerror = () => reject(new Error("webcodecs ws error"));
      sock.onclose = () => statusCb?.("closed");
    });
  }

  function onMessage(e: MessageEvent): void {
    if (typeof e.data === "string") {
      let m: { op?: string; topic?: string; codec?: string };
      try {
        m = JSON.parse(e.data);
      } catch {
        return;
      }
      if (m.op === "video-config" && m.topic) configure(m.topic, m.codec);
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
    decodeChunk(topic, payload, (flags & 1) === 1, tsUs);
  }

  function configure(topic: string, codec?: string): void {
    const c = codec || "avc1.42E01F";
    const prev = decoders.get(topic);
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
        frameCb?.(topic, frame, meta); // caller draws then closes the frame
      },
      error: () => {
        const d = decoders.get(topic);
        if (d) d.sawKey = false; // resync on the next keyframe
      },
    });
    // Don't force hardwareAcceleration:"prefer-hardware": some Chrome contexts configure() OK but then
    // error async with no frames; let the browser choose (still uses hw when available).
    try {
      decoder.configure({ codec: c, optimizeForLatency: true });
    } catch {
      try {
        decoder.configure({ codec: c }); // codec alone is enough for Annex-B in-band SPS/PPS
      } catch {
        return;
      }
    }
    decoders.set(topic, { decoder, sawKey: false, meta });
  }

  function decodeChunk(topic: string, data: Uint8Array, isKey: boolean, tsUs: number): void {
    const d = decoders.get(topic);
    if (!d) return; // config (→ decoder) always precedes chunks
    if (!d.sawKey && !isKey) return; // can't start mid-GOP
    d.sawKey = true;
    try {
      d.decoder.decode(
        new EncodedVideoChunk({ type: isKey ? "key" : "delta", timestamp: tsUs, data }),
      );
    } catch {
      d.sawKey = false; // wait for the next keyframe and resync
    }
  }

  function subscribe(streamId: string): void {
    wanted.add(streamId);
    if (ws?.readyState === WebSocket.OPEN) send({ op: "webcodecs-start", topic: streamId });
  }

  function unsubscribe(streamId: string): void {
    wanted.delete(streamId);
    send({ op: "webcodecs-stop", topic: streamId });
    const d = decoders.get(streamId);
    if (d) {
      try {
        d.decoder.close();
      } catch {
        /* already closed */
      }
      decoders.delete(streamId);
    }
  }

  function send(obj: unknown): void {
    if (ws?.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
  }

  function close(): void {
    for (const d of decoders.values()) {
      try {
        d.decoder.close();
      } catch {
        /* already closed */
      }
    }
    decoders.clear();
    wanted.clear();
    ws?.close();
  }

  return {
    caps,
    label: "webcodecs (h264)",
    connect,
    subscribe,
    unsubscribe,
    onStream() {}, // n/a — this channel is "frames"
    onFrame(cb: (id: string, frame: VideoFrame | ImageBitmap, m: VideoMeta) => void): void {
      frameCb = cb;
    },
    onStatus(cb: (s: Status) => void): void {
      statusCb = cb;
    },
    close,
  };
};
