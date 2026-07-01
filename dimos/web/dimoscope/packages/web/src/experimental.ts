// @dimos/web/experimental — research / benchmark / fallback transports.
//
// Production apps use `ws()` (default) or `webtransport()` from the main "@dimos/web" entry.
// The transports here are NOT the recommended path — they exist for the benchmark suite and as
// niche fallbacks: raw WebRTC-data, SSE, HTTP long-poll, and the raw WebTransport. Each is exposed as a
// `TransportFactory` (for `createDimosClient({ transport }).connect(url)`, uniform with `ws()`), plus the
// raw `createXTransport` for advanced use.
import { createSseTransport, type SseDeps } from "./transports/experimental/sse.ts";
import { createHttpPollTransport, type HttpPollDeps } from "./transports/experimental/httpPoll.ts";
import {
  createWebRtcDataTransport,
  type WebRtcDataDeps,
} from "./transports/experimental/webRtcData.ts";
import { createWebTransportTransport, type WebTransportDeps } from "./transports/webTransport.ts";
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
/** Raw WebTransport (no WS fallback). Prefer the composite `webtransport()` from the main entry, which
 *  auto-falls back to WS. `connect(url)` = the QUIC endpoint; pass `certHashUrl` in opts. */
export const webtransportData = (opts?: Omit<WebTransportDeps, "url">): TransportFactory => (url) =>
  createWebTransportTransport({ url, ...opts });

export {
  createHttpPollTransport,
  createSseTransport,
  createWebRtcDataTransport,
  createWebTransportTransport,
};
export type { HttpPollDeps, SseDeps, WebRtcDataDeps, WebTransportDeps };
