import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import wasm from "vite-plugin-wasm";
import topLevelAwait from "vite-plugin-top-level-await";
import { fileURLToPath } from "node:url";

const r = (p: string) => fileURLToPath(new URL(p, import.meta.url));

// The library lives in ../packages and is consumed by alias (so the example app
// demonstrates the real @dimos/topics + @dimos/react surface). dedupe keeps a
// single React copy across the app + the aliased packages.
export default defineConfig({
  // wasm + topLevelAwait: @eclipse-zenoh/zenoh-ts ships a WASM module via the ESM
  // integration import, which Vite can't handle without these plugins.
  plugins: [wasm(), topLevelAwait(), react()],
  // Exclude WASM-shipping deps from esbuild pre-bundling so Vite serves their files
  // directly: the Rerun viewer (.wasm MIME) and zenoh-ts (vite-plugin-wasm handles it).
  // But DO pre-bundle zenoh-ts's CommonJS deps (e.g. channel-ts) so their named exports
  // resolve when the raw-served zenoh-ts imports them. They're listed as `parent > child`
  // so Vite finds them *through* zenoh-ts — under Deno's node_modules they aren't hoisted
  // to the top level, so a bare "channel-ts" wouldn't resolve.
  optimizeDeps: {
    exclude: ["@rerun-io/web-viewer", "@rerun-io/web-viewer-react", "@eclipse-zenoh/zenoh-ts"],
    include: [
      "@eclipse-zenoh/zenoh-ts > channel-ts",
      "@eclipse-zenoh/zenoh-ts > base64-arraybuffer",
      "@eclipse-zenoh/zenoh-ts > uuid",
      "@eclipse-zenoh/zenoh-ts > tslog",
      "@eclipse-zenoh/zenoh-ts > typed-duration",
      "@eclipse-zenoh/zenoh-ts > @thi.ng/leb128",
    ],
  },
  resolve: {
    dedupe: ["react", "react-dom"],
    alias: {
      "@dimos/topics": r("../packages/topics/src/index.ts"),
      "@dimos/react": r("../packages/react/src/index.tsx"),
    },
  },
  // Multi-page: the app (index.html) + the in-browser benchmark (bench.html). Dev serves
  // any .html directly; build needs both entries listed here.
  build: { rollupOptions: { input: { main: r("./index.html"), bench: r("./bench.html") } } },
  server: { port: 5173, host: true },
});
