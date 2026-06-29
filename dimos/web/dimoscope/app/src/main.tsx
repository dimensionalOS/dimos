import { createRoot } from "react-dom/client";
import { DimosProvider } from "@dimos/react";
import { App } from "./App";
import "./styles.css";

// The dimos-web-gateway serves WebSocket on :8090 (same host you run it on).
const wsUrl = `ws://${location.hostname || "localhost"}:${import.meta.env.VITE_GATEWAY_PORT ?? 8090}`;

createRoot(document.getElementById("root")!).render(
  <DimosProvider url={wsUrl}>
    <App />
  </DimosProvider>,
);
