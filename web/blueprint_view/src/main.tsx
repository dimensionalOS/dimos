import { DimThemeProvider } from "dimos-react";
import { createRoot } from "react-dom/client";
import { App } from "./App.tsx";
import "./view.css";

// Desktop serves its skins' tokens at /theme.css (the view is at /dimos-gateway/blueprint_view there); off Desktop the
// bundled copy themes it.
createRoot(document.getElementById("root")!).render(
  <DimThemeProvider themeUrl="../theme.css">
    <App />
  </DimThemeProvider>,
);
