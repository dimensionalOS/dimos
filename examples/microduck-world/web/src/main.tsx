import { createRoot } from "react-dom/client";
import { registerPanel } from "@dimos/cockpit/panels/registry.tsx";
import { LobbyApp } from "./LobbyApp.tsx";
import "@dimos/cockpit/index.css";
import { WorldPanel } from "./WorldPanel.tsx";
import { BallCameraPanel } from "./BallCameraPanel.tsx";
import { ChatPanel } from "./cockpit/ChatPanel.tsx";
import { TeleopPanel } from "./cockpit/TeleopPanel.tsx";
import "./cockpit/theme.css";
import { WorldControls } from "./WorldControls.tsx";

registerPanel("world3d", WorldPanel);
registerPanel("ball-camera", BallCameraPanel);
registerPanel("control", WorldControls);
registerPanel("chat", ChatPanel);
registerPanel("teleop", TeleopPanel);
const root = createRoot(document.getElementById("root")!);
if (!globalThis.isSecureContext) {
  root.render(
    <p style={{ padding: "1rem" }}>
      Open this page over HTTPS to connect to the world.
    </p>,
  );
} else {
  root.render(<LobbyApp />);
}
