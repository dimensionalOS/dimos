import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { BusProvider } from "./useBus";
import { App } from "./App";
import "./styles.css";

// The bridge serves WebSocket on :8080 (same host you run it on).
const wsUrl = `ws://${location.hostname || "localhost"}:8080`;

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <BusProvider url={wsUrl}>
      <App />
    </BusProvider>
  </StrictMode>,
);
