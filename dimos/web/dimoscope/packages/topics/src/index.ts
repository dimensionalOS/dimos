// @dimos/topics — DimOS topics in the browser (framework-agnostic core).
export { connect, DimosClient } from "./client";
export type { ConnectOpts } from "./client";
export { Topic } from "./topic";
export type { TopicWiring } from "./topic";
export { GatewayWsTransport } from "./adapters/gatewayWs";
// zenoh-ts is browser-only + heavy; the adapter lazy-imports it inside connect(), so
// re-exporting the class here stays cheap (no eager @eclipse-zenoh/zenoh-ts load).
export { ZenohTsTransport } from "./adapters/zenohTs";
export { splitChannel, decodeBody, srcTsMs } from "./decode";
export type { Transport, RawSample, Status, TransportCaps } from "./transport";
export type { MessageMeta, Handler, Subscription, TopicStats, TopicInfo } from "./types";
