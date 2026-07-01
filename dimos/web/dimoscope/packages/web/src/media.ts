// Media plane: pluggable opaque-video delivery (jpeg floor / webrtc / webcodecs), chosen by
// capability negotiation. Unlike Transport it carries no decodable RawSamples — video is just shown.
import type { Status } from "./transport.ts";
import type { DimosClient } from "./client.ts";
import { createJpegTopicMedia } from "./media/jpegTopicMedia.ts";
import { createWebRtcMedia } from "./media/webRtcMedia.ts";
import { createWebCodecsMedia } from "./media/webCodecsMedia.ts";

export type MediaKind = "webcodecs" | "webrtc" | "jpeg";

export interface VideoMeta {
  width: number;
  height: number;
  fps: number;
  codec: string;
}

export interface MediaCaps {
  /** How the app renders: a GPU-composited MediaStream (<video>) vs decoded frames (canvas). */
  output: "stream" | "frames";
  codec: "h264" | "vp8" | "av1" | "jpeg";
  /** Subscribing actually starts/stops bytes (jpeg maps to topic on-demand). */
  onDemand: boolean;
  /** Can carry many concurrent streams (the multi-cam grid). */
  multiStream: boolean;
  hardwareDecode: boolean;
}

/** streamId is the camera topic, e.g. "/dimos/color_image". */
export interface MediaChannel {
  connect(): Promise<void>;
  close(): void;
  subscribe(streamId: string): void;
  unsubscribe(streamId: string): void;
  // An impl fires exactly one of these, per caps.output.
  onStream(cb: (streamId: string, stream: MediaStream) => void): void; // "stream"
  onFrame(cb: (streamId: string, frame: VideoFrame | ImageBitmap, m: VideoMeta) => void): void; // "frames"
  onStatus(cb: (s: Status) => void): void;
  readonly caps: MediaCaps;
  label?: string;
}

export interface MediaDeps {
  client: DimosClient; // for the jpeg-topic floor (subscribes via client.topic)
  gatewayUrl?: string; // media node WS (WebRTC signaling / WebCodecs chunks, e.g. ws://host:8092)
  serverMedia?: readonly MediaKind[]; // what the media node serves; floor = ["jpeg"]
  prefer?: MediaKind[]; // preference order; default ["webrtc","jpeg"]
}

/** Pick the best path the server offers and the browser supports, in preference order; jpeg is the floor (always returns something). */
export function selectMediaChannel(d: MediaDeps): MediaChannel {
  const prefer = d.prefer ?? ["webrtc", "jpeg"];
  const offered = new Set<MediaKind>(d.serverMedia ?? ["jpeg"]);
  const jpeg = () => createJpegTopicMedia({ client: d.client }); // jpeg floor (no gatewayUrl)
  // Browser support per kind (webcodecs chunks ride the WS, so decoder types suffice).
  const supported: Record<MediaKind, boolean> = {
    webcodecs: "VideoDecoder" in globalThis && "EncodedVideoChunk" in globalThis,
    webrtc: "RTCPeerConnection" in globalThis,
    jpeg: true,
  };
  // webcodecs/webrtc need the media node URL, else they skip.
  const build: Record<MediaKind, () => MediaChannel | undefined> = {
    webcodecs: () => d.gatewayUrl ? createWebCodecsMedia({ gatewayUrl: d.gatewayUrl }) : undefined,
    webrtc: () => d.gatewayUrl ? createWebRtcMedia({ gatewayUrl: d.gatewayUrl }) : undefined,
    jpeg,
  };
  for (const kind of prefer) {
    if (!offered.has(kind) || !supported[kind]) continue;
    const ch = build[kind]();
    if (ch) return ch;
  }
  return jpeg();
}
