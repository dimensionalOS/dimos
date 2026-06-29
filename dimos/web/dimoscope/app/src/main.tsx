import { createRoot } from "react-dom/client";
import { DimosProvider } from "@dimos/react";
import { App } from "./App";
import "./styles.css";

// The dimos-web-gateway serves WebSocket on :8090 (same host you run it on).
// NOTE: 8088, not 8090 — DimSim's internal bridge claims :8090 (dimsim_port) and
// kills whatever holds it on startup, so the dimoscope gateway lives on :8088.
const wsUrl = `ws://${location.hostname || "localhost"}:${import.meta.env.VITE_GATEWAY_PORT ?? 8088}`;

createRoot(document.getElementById("root")!).render(
  <DimosProvider url={wsUrl}>
    <App />
  </DimosProvider>,
);
