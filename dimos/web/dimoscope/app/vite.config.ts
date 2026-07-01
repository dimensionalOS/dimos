import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { fileURLToPath } from "node:url";

const r = (p: string) => fileURLToPath(new URL(p, import.meta.url));

// The library lives in ../packages and is consumed by alias, so the app demonstrates the real
// @dimos/web + @dimos/react surface. dedupe keeps one React copy across the app + aliased packages.
export default defineConfig({
  plugins: [react()],
  resolve: {
    dedupe: ["react", "react-dom"],
    alias: {
      "@dimos/web/experimental": r("../packages/web/src/transports/experimental/index.ts"),
      "@dimos/web": r("../packages/web/src/index.ts"),
      "@dimos/react": r("../packages/react/src/index.tsx"),
    },
  },
  server: {
    port: 5173,
    host: true,
    fs: { allow: [r("..")] },
  },
});
