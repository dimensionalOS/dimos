import { useMemo, useState } from "react";
import { createRoot } from "react-dom/client";
import { DimosProvider, type ServerOpt } from "@dimos/react";
import { createAutoTransport, createDimosClient, createGatewayWsTransport } from "@dimos/web";
import { App } from "./App";
import { GatewayContext, normalizeGateway, pushRecentGateway } from "./gateway";
import { setUrlParam } from "./urlState";
import "./styles.css";

// Data plane, camera /media, and bench transports share one host:port; the gateway (host:port) is a
// user-editable setting. WebTransport is the exception — QUIC can't share the HTTP port, so it uses
// WT_PORT=8443; ?wt=<port> overrides it (deployments where only e.g. UDP 443 is reachable).
const GW_PORT = 8080; // gateway HTTP/WS port (python -m gateway default)
const WT_PORT = Number(new URLSearchParams(location.search).get("wt") ?? 8443); // gateway WebTransport/QUIC port

// Initial gateway: ?gw=host:port seeds it, else localStorage, else hostname:GW_PORT.
// Normalized at every entry: pasted full URLs (?gw=http://host:8080/) become host:port.
function initialGateway(): string {
  return normalizeGateway(new URLSearchParams(location.search).get("gw") ?? "") ||
    normalizeGateway(localStorage.getItem("dimos.gw") ?? "") ||
    `${location.hostname}:${GW_PORT}`;
}

// Initial transport: ?transport=<id> seeds the dropdown, else localStorage, else auto. Unknown ids
// fall back to the first server entry inside DimosProvider.
function initialTransport(): string {
  return new URLSearchParams(location.search).get("transport") ??
    localStorage.getItem("dimos.transport") ??
    "auto";
}

// Rebuilt whenever the gateway changes; DimosProvider reconnects on list-identity change.
function buildServers(gateway: string): ServerOpt[] {
  const httpBase = `${location.protocol}//${gateway}`; // http(s)://host:port (SSE/poll/cert base)
  const wsProto = location.protocol === "https:" ? "wss" : "ws";
  const wsBase = `${wsProto}://${gateway}`;
  const media = { gatewayUrl: `${wsBase}/media`, kinds: ["webcodecs", "webrtc", "jpeg"] as const };
  return [
    // Default: one WebTransport connection (data + teleop/rpc over QUIC), falls back to WebSocket.
    {
      id: "auto",
      label: "Auto (WT→WS)",
      connect: async () => {
        const c = createDimosClient({ transport: (url) => createAutoTransport({ url, wtPort: WT_PORT }) });
        await c.connect(gateway);
        return c;
      },
      media: { ...media },
    },
    {
      id: "ws",
      label: "WebSocket",
      url: `${wsBase}/ws`,
      connect: async () => {
        const c = createDimosClient({ transport: (url) => createGatewayWsTransport({ url }) });
        await c.connect(`${wsBase}/ws`);
        return c;
      },
      media: { ...media },
    },
    // Bench delivery mechanisms (read-only), lazily loaded so their browser-only APIs load only when picked.
    {
      id: "sse",
      label: "SSE",
      connect: async () => {
        const { createSseTransport } = await import("@dimos/web/experimental");
        const c = createDimosClient({ transport: (url) => createSseTransport({ url }) });
        await c.connect(httpBase);
        return c;
      },
      media: { ...media },
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
      media: { ...media },
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
      media: { ...media },
    },
    {
      id: "webtransport",
      label: "WebTransport",
      connect: async () => {
        const { createWebTransportTransport } = await import("@dimos/web/experimental");
        const c = createDimosClient({
          transport: (url) => createWebTransportTransport({ url, certHashUrl: `${httpBase}/cert` }),
        });
        await c.connect(`https://${gateway.split(":")[0]}:${WT_PORT}`);
        return c;
      },
      media: { ...media },
    },
  ];
}

function Root() {
  const [gateway, setGatewayState] = useState(initialGateway);
  const servers = useMemo(() => buildServers(gateway), [gateway]);
  const setGateway = (g: string) => {
    const v = normalizeGateway(g);
    if (!v || v === gateway) return;
    pushRecentGateway(v);
    localStorage.setItem("dimos.gw", v);
    setUrlParam("gw", v);
    setGatewayState(v); // → servers rebuild → DimosProvider closes the old client + reconnects to `v`
  };
  const onTransportChange = (id: string) => {
    localStorage.setItem("dimos.transport", id);
    setUrlParam("transport", id);
  };
  return (
    <GatewayContext.Provider value={{ gateway, setGateway }}>
      <DimosProvider
        servers={servers}
        initialServerId={initialTransport()}
        onActiveChange={onTransportChange}
      >
        <App />
      </DimosProvider>
    </GatewayContext.Provider>
  );
}

createRoot(document.getElementById("root")!).render(<Root />);
