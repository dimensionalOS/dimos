import { createRoot } from "react-dom/client";
import { DimosProvider, type ServerOpt } from "@dimos/react";
import { createDimosClient, ws } from "@dimos/topics";
import { App } from "./App";
import "./styles.css";

// Single service: the whole stack (serve.py) is served from ONE origin — the data plane, the camera,
// and the bench transports all live on paths of this same host:port. So everything is same-origin by
// default; the topbar dropdown only switches the DELIVERY MECHANISM (WebSocket vs SSE vs poll vs
// WebRTC-data vs WebTransport), not the host.
//
// `?gw=host:port` still overrides the origin for bad-network testing through a netsim proxy. The
// WebTransport QUIC listener is the one exception to same-origin: QUIC is UDP and can't share the HTTP
// TCP port, so it has its own port (serve.py WT_PORT, default 8443); its cert hash is fetched from
// /cert on the main origin.
const gwOverride = new URLSearchParams(location.search).get("gw");
const origin = gwOverride ?? location.host; // host:port the page was served from
const httpBase = `${location.protocol}//${origin}`; // http(s)://host:port (SSE/poll base)
const wsProto = location.protocol === "https:" ? "wss" : "ws";
const wsBase = `${wsProto}://${origin}`;
const WT_PORT = 8443;

// The camera always rides /media (its own WS path on the same service); kinds are negotiated.
const MEDIA = { gatewayUrl: `${wsBase}/media`, kinds: ["webcodecs", "webrtc", "jpeg"] as const };

const servers: ServerOpt[] = [
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
      const { sse } = await import("@dimos/topics/experimental");
      const c = createDimosClient({ transport: sse() });
      await c.connect(httpBase);
      return c;
    },
    media: { ...MEDIA },
  },
  {
    id: "poll",
    label: "HTTP poll",
    connect: async () => {
      const { poll } = await import("@dimos/topics/experimental");
      const c = createDimosClient({ transport: poll() });
      await c.connect(httpBase);
      return c;
    },
    media: { ...MEDIA },
  },
  {
    id: "webrtc",
    label: "WebRTC data",
    connect: async () => {
      const { webrtc } = await import("@dimos/topics/experimental");
      const c = createDimosClient({ transport: webrtc() });
      await c.connect(`${wsBase}/rtc`);
      return c;
    },
    media: { ...MEDIA },
  },
  {
    id: "webtransport",
    label: "WebTransport",
    connect: async () => {
      const { webtransportData } = await import("@dimos/topics/experimental");
      const c = createDimosClient({
        transport: webtransportData({ certHashUrl: `${httpBase}/cert` }),
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
