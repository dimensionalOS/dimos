import { createRoot } from "react-dom/client";
import { App } from "./App.tsx";
import { startSession } from "./session/session.ts";
import "./index.css";

const root = createRoot(document.getElementById("root")!);

// Same capability checks as the debug page: WebTransport needs a secure
// context, and Safari has no WebTransport as of mid-2026.
if (!globalThis.isSecureContext) {
  root.render(
    <p style={{ padding: "1rem" }}>
      Not a secure context: WebTransport needs https:// or http://localhost.
    </p>,
  );
} else if (!("WebTransport" in globalThis)) {
  root.render(
    <p style={{ padding: "1rem" }}>
      This browser has no WebTransport support. Use Chromium or Firefox.
    </p>,
  );
} else {
  const session = startSession();
  root.render(<App session={session} />);
}
