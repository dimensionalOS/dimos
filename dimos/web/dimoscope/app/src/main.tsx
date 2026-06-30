import { createRoot } from "react-dom/client";
import { DimosProvider, type ServerOpt } from "@dimos/react";
import { connect } from "@dimos/topics";
import { App } from "./App";
import "./styles.css";

// Transports, each on its own port; the topbar dropdown picks the active one.
// Bun↔LCM uses :8089 here (not :8090) — DimSim's internal bridge claims :8090 and
// kills whatever holds it. start-all.sh launches all of these together.
const host = location.hostname || "localhost";
// Bad-network testing in the browser: `?gw=host:port` routes the WS DATA gateways through a netsim
// proxy, so the real app degrades like the headless `bench:matrix` profiles. Run e.g.
//   NETSIM_TARGET=localhost:8089 NETSIM_LISTEN=8099 deno run -A bench/netsim.ts
// then open `…/?gw=localhost:8099` and pick the matching transport in the dropdown. The MEDIA plane
// (:8092, WebRTC) stays direct — a TCP proxy can't shape UDP; use macOS Network Link Conditioner for
// the camera/datachannel. See bench/netsim.ts.
const gwOverride = new URLSearchParams(location.search).get("gw");
const gw = (port: number) => gwOverride ?? `${host}:${port}`;
// The MEDIA plane always rides the standalone media node (:8092) — its own Zenoh peer, so the camera
// gets WebRTC / WebCodecs regardless of which transport carries the data plane. Data + teleop/goal/RPC
// stay on the gateway (:8088 / :8089 / zenoh-ts control), so the trust boundary is unchanged.
const MEDIA = { gatewayUrl: `ws://${host}:8092`, kinds: ["webcodecs", "webrtc", "jpeg"] as const };
const servers: ServerOpt[] = [
  {
    id: "zenoh",
    label: "Python↔Zenoh",
    url: `ws://${gw(8088)}`,
    connect: () => connect({ url: `ws://${gw(8088)}` }),
    media: { ...MEDIA },
  },
  {
    id: "lcm",
    label: "Bun↔LCM",
    url: `ws://${gw(8089)}`,
    connect: () => connect({ url: `ws://${gw(8089)}` }),
    media: { ...MEDIA },
  },
  {
    id: "zenoh-ts",
    label: "zenoh-ts (direct)",
    // Lazy: zenoh-ts + its bundle only load if this option is picked, so a load
    // failure can't break the default gateway paths. Read path = direct Zenoh via the
    // remote-api bridge (:10000); teleop/goal go through the gateway (:8088) so its
    // clamp + deadman safety still applies.
    connect: async () => {
      const { createZenohTsTransport } = await import("@dimos/topics");
      return connect({
        transport: createZenohTsTransport({
          remoteApiUrl: `ws://${gw(10000)}`,
          controlUrl: `ws://${gw(8088)}`,
        }),
      });
    },
    media: { ...MEDIA },
  },
];

createRoot(document.getElementById("root")!).render(
  <DimosProvider servers={servers}>
    <App />
  </DimosProvider>,
);
