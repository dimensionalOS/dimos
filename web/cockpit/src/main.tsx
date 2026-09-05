// Stylesheets first so the module styles imported by App win ties with the
// page-wide defaults (bundle order follows import order).
import "@fontsource-variable/inter/wght.css";
import "@fontsource-variable/jetbrains-mono/wght.css";
import "./index.css";
import { createRoot } from "react-dom/client";
import { connect } from "@dimos/sdk";
import { App } from "./App.tsx";
import { cockpitDecoders, installAutoSubscriptions } from "./subscriptions.ts";

const root = createRoot(document.getElementById("root")!);

// Capability checks before anything mounts: WebTransport needs a secure
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
  const session = connect({ decoders: cockpitDecoders });
  installAutoSubscriptions(session);
  root.render(<App session={session} />);
}
