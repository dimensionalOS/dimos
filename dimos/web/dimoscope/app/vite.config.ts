import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { fileURLToPath } from "node:url";

const r = (p: string) => fileURLToPath(new URL(p, import.meta.url));

// The library lives in ../packages and is consumed by alias (so the example app
// demonstrates the real @dimos/topics + @dimos/react surface). dedupe keeps a
// single React copy across the app + the aliased packages.
export default defineConfig({
  plugins: [react()],
  resolve: {
    dedupe: ["react", "react-dom"],
    alias: {
      "@dimos/topics": r("../packages/topics/src/index.ts"),
      "@dimos/react": r("../packages/react/src/index.tsx"),
    },
  },
  server: { port: 5173, host: true },
});
