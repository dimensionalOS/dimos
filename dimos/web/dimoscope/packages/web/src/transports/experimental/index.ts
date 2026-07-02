// @dimos/web/experimental — non-production delivery mechanisms (benchmark / niche fallback), exposed as
// raw `createXTransport(deps): Transport` constructors like every transport. Production uses the
// default gateway WebSocket or createAutoTransport.
export { createSseTransport, type SseDeps } from "./sse.ts";
export { createHttpPollTransport, type HttpPollDeps } from "./httpPoll.ts";
export { createWebRtcDataTransport, type WebRtcDataDeps } from "./webRtcData.ts";
export { createWebTransportTransport, type WebTransportDeps } from "../webTransport.ts";
