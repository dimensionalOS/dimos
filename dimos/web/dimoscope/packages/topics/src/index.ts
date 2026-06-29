// @dimos/topics — DimOS topics in the browser (framework-agnostic core).
export { connect, DimosClient } from "./client";
export type { ConnectOpts } from "./client";
export { Topic } from "./topic";
export type { TopicWiring } from "./topic";
export { GatewayWsTransport } from "./adapters/gatewayWs";
export { splitChannel, decodeBody, srcTsMs } from "./decode";
export type { Transport, RawSample, Status, TransportCaps } from "./transport";
export type { MessageMeta, Handler, Subscription, TopicStats, TopicInfo } from "./types";
