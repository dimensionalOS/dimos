import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import wasm from "vite-plugin-wasm";
import topLevelAwait from "vite-plugin-top-level-await";
import { fileURLToPath } from "node:url";

const r = (p: string) => fileURLToPath(new URL(p, import.meta.url));

// The library lives in ../packages and is consumed by alias (so the example app
// demonstrates the real @dimos/web + @dimos/react surface). dedupe keeps a
// single React copy across the app + the aliased packages.
export default defineConfig({
  // wasm + topLevelAwait: the Rerun web-viewer ships a WASM module via an ESM integration
  // import, which Vite can't handle without these plugins.
  plugins: [wasm(), topLevelAwait(), react()],
  // Exclude the WASM-shipping Rerun viewer from esbuild pre-bundling so Vite serves its
  // files directly (the .wasm MIME).
  optimizeDeps: {
    exclude: ["@rerun-io/web-viewer", "@rerun-io/web-viewer-react"],
  },
  resolve: {
    dedupe: ["react", "react-dom"],
    alias: {
      "@dimos/web/experimental": r("../packages/web/src/experimental.ts"),
      "@dimos/web": r("../packages/web/src/index.ts"),
      "@dimos/react": r("../packages/react/src/index.tsx"),
    },
  },
  server: { port: 5173, host: true },
});
