// @dimos/web — DimOS topics in the browser. Entry: `createDimosClient({ transport? }).connect(url)`.
// ws() = default (universal WS backbone + control plane); webtransport() = data+control over QUIC with a
// transparent WS fallback. Research/bench transports (WebRTC-data, SSE, HTTP-poll, raw WebTransport) live
// in "@dimos/web/experimental".
export { createDimosClient, ws } from "./client.ts";
export type { DimosClient, DimosClientDeps, ModuleMap, TransportFactory } from "./client.ts";
export { webtransport } from "./transports/composite.ts";
export type { WebtransportOpts } from "./transports/composite.ts";
export { createTopic } from "./topic.ts";
export type { Topic, TopicDeps, TopicWiring } from "./topic.ts";
export { createGatewayWsTransport } from "./transports/gatewayWs.ts";
export type { GatewayWsDeps } from "./transports/gatewayWs.ts";
export { b64ToBytes, frameToSample } from "./transports/frame.ts";
export { formatMarkdown, measureScenario, onDemandSaving, STREAM_PROFILES } from "./bench.ts";
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
export { selectMediaChannel } from "./media.ts";
export { createJpegTopicMedia } from "./media/jpegTopicMedia.ts";
export type { JpegTopicMediaDeps } from "./media/jpegTopicMedia.ts";
export { createWebRtcMedia } from "./media/webRtcMedia.ts";
export type { WebRtcMediaDeps } from "./media/webRtcMedia.ts";
export { createWebCodecsMedia } from "./media/webCodecsMedia.ts";
export type { WebCodecsMediaDeps } from "./media/webCodecsMedia.ts";
export type { MediaCaps, MediaChannel, MediaDeps, MediaKind, VideoMeta } from "./media.ts";
