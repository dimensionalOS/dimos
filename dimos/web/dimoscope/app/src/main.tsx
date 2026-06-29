import { createRoot } from "react-dom/client";
import { DimosProvider, type ServerOpt } from "@dimos/react";
import { connect } from "@dimos/topics";
import { App } from "./App";
import "./styles.css";

// Transports, each on its own port; the topbar dropdown picks the active one.
// Bun↔LCM uses :8089 here (not :8090) — DimSim's internal bridge claims :8090 and
// kills whatever holds it. start-all.sh launches all of these together.
const host = location.hostname || "localhost";
// The MEDIA plane always rides the Python aiortc gateway (:8088): it reads the same bus, so the
// camera gets WebRTC / WebCodecs regardless of which transport carries the data plane. (Before,
// only Python↔Zenoh advertised media, so picking webrtc on lcm/zenoh-ts silently fell to jpeg.)
const MEDIA = { gatewayUrl: `ws://${host}:8088`, kinds: ["webcodecs", "webrtc", "jpeg"] as const };
const servers: ServerOpt[] = [
  {
    id: "zenoh",
    label: "Python↔Zenoh",
    connect: () => connect({ url: `ws://${host}:8088` }),
    media: { ...MEDIA },
  },
  {
    id: "lcm",
    label: "Bun↔LCM",
    connect: () => connect({ url: `ws://${host}:8089` }),
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
      const { ZenohTsTransport } = await import("@dimos/topics");
      return connect({ transport: new ZenohTsTransport(`ws://${host}:10000`, `ws://${host}:8088`) });
    },
    media: { ...MEDIA },
  },
];

createRoot(document.getElementById("root")!).render(
  <DimosProvider servers={servers}>
    <App />
  </DimosProvider>,
);
