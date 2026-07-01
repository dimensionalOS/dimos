import { createRoot } from "react-dom/client";
import { DimosProvider, type ServerOpt } from "@dimos/react";
import { createDimosClient, webtransport, ws } from "@dimos/web";
import { App } from "./App";
import "./styles.css";

// Single service: the whole stack (serve.py) is served from ONE origin — the data plane, the camera,
// and the bench transports all live on paths of this same host:port. So everything is same-origin by
// default; the topbar dropdown only switches the DELIVERY MECHANISM (WebSocket vs SSE vs poll vs
// WebRTC-data vs WebTransport), not the host.
//
// `?gw=host:port` still overrides the origin for bad-network testing against a remote VPS. The
// WebTransport QUIC listener is the one exception to same-origin: QUIC is UDP and can't share the HTTP
// TCP port, so it has its own port (serve.py WT_PORT, default 8443); its cert hash is fetched from
// /cert on the main origin.
// Gateway HTTP/WS port is hard-coded (serve.py default 8080) so every transport hits the gateway
// regardless of where the app is served — Vite dev (:5173), prod (:8080), or a LAN IP. `?gw` overrides.
const GW_PORT = 8080;
const gwOverride = new URLSearchParams(location.search).get("gw");
const origin = gwOverride ?? `${location.hostname}:${GW_PORT}`; // gateway host:port (NOT location.host)
const httpBase = `${location.protocol}//${origin}`; // http(s)://host:port (SSE/poll base)
const wsProto = location.protocol === "https:" ? "wss" : "ws";
const wsBase = `${wsProto}://${origin}`;
const WT_PORT = 8443;

// The camera always rides /media (its own WS path on the same service); kinds are negotiated.
const MEDIA = { gatewayUrl: `${wsBase}/media`, kinds: ["webcodecs", "webrtc", "jpeg"] as const };

const servers: ServerOpt[] = [
  // Default: prefer ONE WebTransport connection (data + teleop/rpc over QUIC, no HoL), fall back to a
  // plain WebSocket when WT is unavailable or can't connect (Safari, UDP-blocked). Teleop always works.
  {
    id: "auto",
    label: "Auto (WT→WS)",
    connect: async () => {
      const c = createDimosClient({ transport: webtransport() });
      await c.connect(origin);
      return c;
    },
    media: { ...MEDIA },
  },
  {
    id: "ws",
    label: "WebSocket",
    url: `${wsBase}/ws`,
    connect: async () => {
      const c = createDimosClient({ transport: ws() });
      await c.connect(`${wsBase}/ws`);
      return c;
    },
    media: { ...MEDIA },
  },
  // Bench delivery mechanisms (read-only) — same frames, same origin, different transport. Lazy so
  // their browser-only APIs (RTCPeerConnection, WebTransport) load only when picked.
  {
    id: "sse",
    label: "SSE",
    connect: async () => {
      const { createSseTransport } = await import("@dimos/web/experimental");
      const c = createDimosClient({ transport: (url) => createSseTransport({ url }) });
      await c.connect(httpBase);
      return c;
    },
    media: { ...MEDIA },
  },
  {
    id: "poll",
    label: "HTTP poll",
    connect: async () => {
      const { createHttpPollTransport } = await import("@dimos/web/experimental");
      const c = createDimosClient({ transport: (url) => createHttpPollTransport({ url }) });
      await c.connect(httpBase);
      return c;
    },
    media: { ...MEDIA },
  },
  {
    id: "webrtc",
    label: "WebRTC data",
    connect: async () => {
      const { createWebRtcDataTransport } = await import("@dimos/web/experimental");
      const c = createDimosClient({ transport: (url) => createWebRtcDataTransport({ url }) });
      await c.connect(`${wsBase}/rtc`);
      return c;
    },
    media: { ...MEDIA },
  },
  {
    id: "webtransport",
    label: "WebTransport",
    connect: async () => {
      const { createWebTransportTransport } = await import("@dimos/web/experimental");
      const c = createDimosClient({
        transport: (url) => createWebTransportTransport({ url, certHashUrl: `${httpBase}/cert` }),
      });
      await c.connect(`https://${origin.split(":")[0]}:${WT_PORT}`);
      return c;
    },
    media: { ...MEDIA },
  },
];

createRoot(document.getElementById("root")!).render(
  <DimosProvider servers={servers}>
    <App />
  </DimosProvider>,
);
