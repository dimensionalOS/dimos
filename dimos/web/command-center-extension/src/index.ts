import { PanelExtensionContext, ExtensionContext } from "@foxglove/extension";

import { initializeApp } from "./init";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "command-center", initPanel });
}

export function initPanel(context: PanelExtensionContext): () => void {
  initializeApp(context.panelElement);
  return () => {
    // Cleanup function
  };
}
