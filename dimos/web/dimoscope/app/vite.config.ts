import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { fileURLToPath } from "node:url";

const r = (p: string) => fileURLToPath(new URL(p, import.meta.url));

// The library lives in ../packages and is consumed by alias (so the example app
// demonstrates the real @dimos/topics + @dimos/react surface). dedupe keeps a
// single React copy across the app + the aliased packages.
export default defineConfig({
  plugins: [react()],
  // The Rerun web viewer ships a .wasm that Vite's dep pre-bundler serves with
  // the wrong MIME type ("Incorrect response MIME type. Expected application/wasm").
  // Excluding it makes Vite serve the package files directly with correct headers.
  optimizeDeps: { exclude: ["@rerun-io/web-viewer", "@rerun-io/web-viewer-react"] },
  resolve: {
    dedupe: ["react", "react-dom"],
    alias: {
      "@dimos/topics": r("../packages/topics/src/index.ts"),
      "@dimos/react": r("../packages/react/src/index.tsx"),
    },
  },
  server: { port: 5173, host: true },
});
