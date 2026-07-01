// @dimos/topics — DimOS topics in the browser (framework-agnostic core).
//
// Entry: `createDimosClient({ transport? }).connect(url)`. Production transports are `ws()` (default,
// the universal WS backbone + control plane) and `webtransport()` (WS control + WebTransport data — no
// head-of-line blocking under loss). The research/benchmark transports (raw WebRTC-data, SSE, HTTP-poll,
// raw WebTransport, zenoh-ts) live in "@dimos/topics/experimental".
export { createDimosClient, ws, wsServerJson } from "./client.ts";
export type {
  DimosClient,
  DimosClientDeps,
  ModuleMap,
  TransportFactory,
} from "./client.ts";
export { webtransport } from "./adapters/composite.ts";
export type { WebtransportOpts } from "./adapters/composite.ts";
export { createTopic } from "./topic.ts";
export type { Topic, TopicDeps, TopicWiring } from "./topic.ts";
export { createGatewayWsTransport } from "./adapters/gatewayWs.ts";
export type { GatewayWsDeps } from "./adapters/gatewayWs.ts";
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
  Message,
  MessageMeta,
  Qos,
  QosCaps,
  Subscription,
  TopicInfo,
  TopicStats,
} from "./types.ts";
// QoS lanes — ROS 2-grounded named profiles (sane defaults, configurable) + the scheduler priority map.
export { applyCaps, defaultLane, LANES, PRIORITY_RANK, resolveQos } from "./qos.ts";
export type { Lane } from "./qos.ts";
// Media plane — pluggable, negotiated video delivery (beside Transport).
export { browserSupports, selectMediaChannel } from "./media.ts";
export { createJpegTopicMedia } from "./adapters/jpegTopicMedia.ts";
export type { JpegTopicMediaDeps } from "./adapters/jpegTopicMedia.ts";
export { createWebRtcMedia } from "./adapters/webRtcMedia.ts";
export type { WebRtcMediaDeps } from "./adapters/webRtcMedia.ts";
export { createWebCodecsMedia } from "./adapters/webCodecsMedia.ts";
export type { WebCodecsMediaDeps } from "./adapters/webCodecsMedia.ts";
export type { MediaCaps, MediaChannel, MediaDeps, MediaKind, VideoMeta } from "./media.ts";
