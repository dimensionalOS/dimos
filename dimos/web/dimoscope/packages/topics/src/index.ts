// @dimos/topics — DimOS topics in the browser (framework-agnostic core).
export { connect, createDimosClient } from "./client.ts";
export type { ConnectOpts, DimosClient, DimosClientDeps } from "./client.ts";
export { createTopic } from "./topic.ts";
export type { Topic, TopicDeps, TopicWiring } from "./topic.ts";
export { createGatewayWsTransport } from "./adapters/gatewayWs.ts";
export type { GatewayWsDeps } from "./adapters/gatewayWs.ts";
// zenoh-ts is browser-only + heavy; the adapter lazy-imports it inside connect(), so
// re-exporting the factory here stays cheap (no eager @eclipse-zenoh/zenoh-ts load).
export { createZenohTsTransport } from "./adapters/zenohTs.ts";
export type { ZenohTsDeps } from "./adapters/zenohTs.ts";
// Read-only delivery mechanisms over the same gateway bus tap — for the data-path
// benchmark (WebSocket vs SSE vs HTTP long-poll, same frames, different transport).
export { createSseTransport } from "./adapters/sse.ts";
export type { SseDeps } from "./adapters/sse.ts";
export { createHttpPollTransport } from "./adapters/httpPoll.ts";
export type { HttpPollDeps } from "./adapters/httpPoll.ts";
// WebRTC DataChannel (browser-only): unordered/lossy delivery via servers/webrtc_data.py — no TCP
// head-of-line blocking. Lazy browser-API use inside connect(), so importing this stays cheap.
export { createWebRtcDataTransport } from "./adapters/webRtcData.ts";
export type { WebRtcDataDeps } from "./adapters/webRtcData.ts";
// WebTransport (HTTP/3 / QUIC, browser-only): datagrams (small) + streams (large), no TCP HoL.
export { createWebTransportTransport } from "./adapters/webTransport.ts";
export type { WebTransportDeps } from "./adapters/webTransport.ts";
export { b64ToBytes, frameToSample } from "./adapters/gatewayFrame.ts";
export { decodeBody, splitChannel, srcTsMs } from "./decode.ts";
export {
  BENCH_SCENARIOS,
  formatMarkdown,
  measureScenario,
  onDemandSaving,
  STREAM_PROFILES,
} from "./bench.ts";
export type { BenchRow, BenchScenario, StreamProfile } from "./bench.ts";
export type { CommandInfo, RawSample, Status, Transport, TransportCaps } from "./transport.ts";
export type {
  Handler,
  MessageMeta,
  Qos,
  QosCaps,
  Subscription,
  TopicInfo,
  TopicStats,
} from "./types.ts";
// Media plane — pluggable, negotiated video delivery (beside Transport).
export { browserSupports, selectMediaChannel } from "./media.ts";
export { createJpegTopicMedia } from "./adapters/jpegTopicMedia.ts";
export type { JpegTopicMediaDeps } from "./adapters/jpegTopicMedia.ts";
export { createWebRtcMedia } from "./adapters/webRtcMedia.ts";
export type { WebRtcMediaDeps } from "./adapters/webRtcMedia.ts";
export { createWebCodecsMedia } from "./adapters/webCodecsMedia.ts";
export type { WebCodecsMediaDeps } from "./adapters/webCodecsMedia.ts";
export { decodeImageToBitmap } from "./image.ts";
export type { MediaCaps, MediaChannel, MediaDeps, MediaKind, VideoMeta } from "./media.ts";
export type { ImageMsg } from "./image.ts";
