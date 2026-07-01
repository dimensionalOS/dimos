// @dimos/topics/experimental — research / benchmark / fallback transports.
//
// Production apps use `ws()` (default) or `webtransport()` from the main "@dimos/topics" entry.
// The transports here are NOT the recommended path — they exist for the benchmark suite and as
// niche fallbacks: raw WebRTC-data, SSE, HTTP long-poll, the raw (data-only) WebTransport, and the
// direct zenoh-ts client. Each is exposed as a `TransportFactory` (for `createDimosClient({ transport })
// .connect(url)`, uniform with `ws()`), plus the raw `createXTransport` for advanced use.
import { createSseTransport, type SseDeps } from "./adapters/sse.ts";
import { createHttpPollTransport, type HttpPollDeps } from "./adapters/httpPoll.ts";
import { createWebRtcDataTransport, type WebRtcDataDeps } from "./adapters/webRtcData.ts";
import { createWebTransportTransport, type WebTransportDeps } from "./adapters/webTransport.ts";
import { createZenohTsTransport, type ZenohTsDeps } from "./adapters/zenohTs.ts";
import type { TransportFactory } from "./client.ts";

/** SSE (read-only, HTTP GET stream). `connect(url)` = the HTTP origin (adapter appends `/sse`). */
export const sse = (opts?: Omit<SseDeps, "url">): TransportFactory => (url) =>
  createSseTransport({ url, ...opts });
/** HTTP long-poll (read-only, universal baseline). `connect(url)` = the HTTP origin. */
export const poll = (opts?: Omit<HttpPollDeps, "url">): TransportFactory => (url) =>
  createHttpPollTransport({ url, ...opts });
/** Raw WebRTC DataChannel (read-only, unordered/lossy). `connect(url)` = the `/rtc` signaling WS. */
export const webrtc = (opts?: Omit<WebRtcDataDeps, "url">): TransportFactory => (url) =>
  createWebRtcDataTransport({ url, ...opts });
/** Raw (data-only) WebTransport — no control. Prefer the composite `webtransport()` from the main
 *  entry. `connect(url)` = the QUIC endpoint; pass `certHashUrl` in opts. */
export const webtransportData = (opts?: Omit<WebTransportDeps, "url">): TransportFactory => (url) =>
  createWebTransportTransport({ url, ...opts });
/** Direct zenoh-ts client. `connect(url)` = the zenoh remote-api WS URL. */
export const zenohTs = (opts?: Omit<ZenohTsDeps, "remoteApiUrl">): TransportFactory => (url) =>
  createZenohTsTransport({ remoteApiUrl: url, ...opts });

export {
  createHttpPollTransport,
  createSseTransport,
  createWebRtcDataTransport,
  createWebTransportTransport,
  createZenohTsTransport,
};
export type { HttpPollDeps, SseDeps, WebRtcDataDeps, WebTransportDeps, ZenohTsDeps };
