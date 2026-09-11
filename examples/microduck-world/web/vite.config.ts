import { fileURLToPath } from "node:url";
import react from "@vitejs/plugin-react";
import { defineConfig } from "vitest/config";

const source = (path: string) => fileURLToPath(new URL(path, import.meta.url));
export default defineConfig({
  base: "/client/",
  plugins: [{
    name: "microduck-cockpit-presentation",
    enforce: "pre",
    resolveId(id, importer) {
      if (id.endsWith("/LayoutTree.tsx") && importer?.includes("/vendor/dimos/web/cockpit/")) return source("./src/cockpit/LayoutTree.tsx");
      if (id.endsWith("/PanelFrame.tsx") && !id.includes("/cockpit/PanelFrame") && importer && (importer.includes("/vendor/dimos/web/cockpit/") || id.startsWith("@dimos/cockpit/"))) return source("./src/cockpit/PanelFrame.tsx");
    },
    transform(code, id) {
      if (!id.includes("/vendor/dimos/web/cockpit/") || !id.endsWith(".css")) return;
      const colors: Record<string,string> = {"#1c2128":"raised","#14171a":"panel","#1b1f24":"raised","#30363d":"line","#d7dde3":"text","#8b949e":"muted","#12283f":"blue","#3d2708":"orange","#0d1117":"input","#21262d":"button","#58a6ff":"link","#2f81f7":"link","#d29922":"warning","#e3b341":"warning","#f85149":"error"};
      return code.replace(/#[0-9a-fA-F]{6}\b/g, color => colors[color.toLowerCase()] ? `var(--mw-${colors[color.toLowerCase()]}, ${color})` : color);
    },
  }, react()],
  resolve: {
    dedupe: ["react", "react-dom"],
    alias: {
      "@dimos/sdk/internal/teleop": source("../vendor/dimos/web/sdk/src/internal/teleopMachine.ts"),
      "@dimos/sdk/react": source("../vendor/dimos/web/sdk/src/react.ts"),
      "@dimos/sdk": source("../vendor/dimos/web/sdk/src/index.ts"),
      "@dimos/shared/manifest": source("../vendor/dimos/web/shared/manifest.ts"),
      "@dimos/shared": source("../vendor/dimos/web/shared/protocol.ts"),
      "@dimos/cockpit": source("../vendor/dimos/web/cockpit/src"),
    },
  },
  test: { pool: "threads", include: ["src/**/*.test.ts"] },
  build: { target: "es2022" },
});
