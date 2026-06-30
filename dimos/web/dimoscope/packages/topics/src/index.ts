// @dimos/topics — DimOS topics in the browser (framework-agnostic core).
export { connect, createDimosClient } from "./client";
export type { ConnectOpts, DimosClient, DimosClientDeps } from "./client";
export { createTopic } from "./topic";
export type { Topic, TopicWiring, TopicDeps } from "./topic";
export { createGatewayWsTransport } from "./adapters/gatewayWs";
export type { GatewayWsDeps } from "./adapters/gatewayWs";
// zenoh-ts is browser-only + heavy; the adapter lazy-imports it inside connect(), so
// re-exporting the factory here stays cheap (no eager @eclipse-zenoh/zenoh-ts load).
export { createZenohTsTransport } from "./adapters/zenohTs";
export type { ZenohTsDeps } from "./adapters/zenohTs";
export { splitChannel, decodeBody, srcTsMs } from "./decode";
export { BENCH_SCENARIOS, measureScenario, formatMarkdown, onDemandSaving } from "./bench";
export type { BenchScenario, BenchRow } from "./bench";
export type { Transport, RawSample, Status, TransportCaps, CommandInfo } from "./transport";
export type { MessageMeta, Handler, Subscription, TopicStats, TopicInfo } from "./types";
// Media plane — pluggable, negotiated video delivery (beside Transport).
export { selectMediaChannel, browserSupports } from "./media";
export { createJpegTopicMedia } from "./adapters/jpegTopicMedia";
export type { JpegTopicMediaDeps } from "./adapters/jpegTopicMedia";
export { createWebRtcMedia } from "./adapters/webRtcMedia";
export type { WebRtcMediaDeps } from "./adapters/webRtcMedia";
export { createWebCodecsMedia } from "./adapters/webCodecsMedia";
export type { WebCodecsMediaDeps } from "./adapters/webCodecsMedia";
export { decodeImageToBitmap } from "./image";
export type { MediaChannel, MediaCaps, MediaKind, MediaDeps, VideoMeta } from "./media";
export type { ImageMsg } from "./image";
