// Media plane selector: picks the best pluggable video channel (jpeg floor / webrtc / webcodecs) by
// capability negotiation. The channel types live in types.ts; the impls in ./media/.
import type { MediaChannel, MediaKind } from "./types.ts";
import type { DimosClient } from "./client.ts";
import { createJpegTopicMedia } from "./media/jpegTopicMedia.ts";
import { createWebRtcMedia } from "./media/webRtcMedia.ts";
import { createWebCodecsMedia } from "./media/webCodecsMedia.ts";

export interface MediaDeps {
  client: DimosClient; // for the jpeg-topic floor (subscribes via client.topic)
  gatewayUrl?: string; // the gateway /media WS (WebRTC signaling / WebCodecs chunks, e.g. ws://host:8080/media)
  serverMedia?: readonly MediaKind[]; // what the gateway's media plane serves; floor = ["jpeg"]
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
  // webcodecs/webrtc need the /media URL, else they skip.
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
