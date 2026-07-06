// @dimos/web — DimOS topics in the browser. Entry: `createDimosClient({ transport? }).connect(url)`.
// Default transport: the gateway WebSocket. Every transport is a raw `create*Transport(deps)`
// constructor — pass one as `transport: (url) => create*Transport({ url, … })`; createAutoTransport =
// data+control over QUIC with a transparent WS fallback. Research/bench transports (WebRTC-data,
// SSE, HTTP-poll, raw WebTransport) live in "@dimos/web/experimental".
export { createDimosClient } from "./client.ts";
export type { DimosClient, DimosClientDeps, ModuleMap, TransportFactory } from "./client.ts";
export { createAutoTransport } from "./transports/composite.ts";
export type { AutoTransportDeps } from "./transports/composite.ts";
export { createTopic } from "./topic.ts";
export type { Topic, TopicDeps, TopicWiring } from "./topic.ts";
export { createGatewayWsTransport } from "./transports/gatewayWs.ts";
export type { GatewayWsDeps } from "./transports/gatewayWs.ts";
export { b64ToBytes, frameToSample } from "./transports/frame.ts";
export {
  BULK_LANES,
  coexProfile,
  defaultGrace,
  defaultWarmup,
  FAST_LANE,
  measureScenario,
  ON_DEMAND_PAIR,
  onDemandSaving,
  POSE_LANES,
  STREAM_PROFILES,
} from "./bench.ts";
export type {
  BenchBucket,
  BenchOpts,
  BenchRow,
  BenchScenario,
  BucketLane,
  GenSpec,
  LaneStats,
  StreamProfile,
} from "./bench.ts";
export {
  buildFloodOffIndex,
  fastLane,
  floodOffKey,
  formatMarkdown,
  interferenceDelta,
  isCoexRow,
  parseRun,
  reviveRun,
  RUN_SCHEMA,
  RUN_VERSION,
  serializeRun,
  stripBuckets,
  trimRuns,
} from "./benchRun.ts";
export type {
  FloodOffStat,
  GeneratorConfig,
  InterferenceDelta,
  RunCell,
  RunMeta,
  RunRecord,
} from "./benchRun.ts";
export type { CommandInfo, RawSample, Status, Transport, TransportCaps } from "./types.ts";
export type {
  ClockSample,
  Handler,
  Message,
  MessageMeta,
  Qos,
  ServerQosField,
  Subscription,
  TopicInfo,
  TopicStats,
} from "./types.ts";
// QoS lanes — ROS 2-grounded named profiles (sane defaults, configurable) + the scheduler priority map.
export { defaultLane, GATEWAY_QOS, LANES, PRIORITY_RANK, resolveQos } from "./qos.ts";
export type { Lane } from "./qos.ts";
// Media plane — pluggable, negotiated video delivery.
export { selectMediaChannel } from "./media.ts";
export { createJpegTopicMedia } from "./media/jpegTopicMedia.ts";
export type { JpegTopicMediaDeps } from "./media/jpegTopicMedia.ts";
export { createWebRtcMedia } from "./media/webRtcMedia.ts";
export type { WebRtcMediaDeps } from "./media/webRtcMedia.ts";
export { createWebCodecsMedia } from "./media/webCodecsMedia.ts";
export type { WebCodecsMediaDeps } from "./media/webCodecsMedia.ts";
export type { MediaCaps, MediaChannel, MediaKind, VideoMeta } from "./types.ts";
export type { MediaDeps } from "./media.ts";

// Draco point-cloud variant (gateway/cloud.py): the custom-type marker + the dep-free envelope parse.
// (Geometry decode for rendering lives in the app — it carries the draco3d wasm dep, not the SDK.)
export { decodeDracoEnvelope, DRACO_TYPE } from "./draco.ts";
export type { DracoCloud } from "./draco.ts";
