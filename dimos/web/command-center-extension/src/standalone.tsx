/**
 * Standalone entry point for the Command Center React app.
 * This allows the command center to run outside of Foxglove as a regular web page.
 */
import * as React from "react";
import { createRoot } from "react-dom/client";

import App from "./App";

const container = document.getElementById("root");
if (container) {
  const root = createRoot(container);
  root.render(
    <React.StrictMode>
      <App />
    </React.StrictMode>
  );
} else {
  console.error("Root element not found");
}
