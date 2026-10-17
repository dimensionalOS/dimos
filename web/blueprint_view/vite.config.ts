/// <reference types="vitest/config" />
import deno from "@deno/vite-plugin";
import react from "@vitejs/plugin-react";
import { defineConfig, type Plugin } from "vite";

// deno() drops the subpath of a jsr package's `npm:react@^19/jsx-runtime`
// import (dimos-react's JSX), so it got React itself: hand those to Vite as
// plain `react/...` instead.
const npmReactSubpaths: Plugin = {
  name: "npm-react-subpaths",
  enforce: "pre",
  resolveId(id, importer, options) {
    const match = /^npm:(react(?:-dom)?)@[^/]+(\/.+)$/.exec(id);
    return match
      ? this.resolve(match[1] + match[2], importer, { ...options, skipSelf: true })
      : null;
  },
};

// The gateway serves index.html at /blueprint_view and the rest under /blueprint_view/ (Desktop puts both
// under /dimos-gateway), so assets are relative and live in blueprint_view/. The dev proxy strips /dimos-gateway like Desktop.
// deno() resolves the jsr packages (dimos-react in deno.json); dedupe
// keeps them on the app's one React.
export default defineConfig({
  plugins: [npmReactSubpaths, deno(), react()],
  base: "./",
  resolve: { dedupe: ["react", "react-dom"] },
  server: {
    proxy: {
      "/dimos-gateway": {
        target: "http://127.0.0.1:5555",
        rewrite: (path) => path.replace(/^\/dimos-gateway/, ""),
      },
      "/api": "http://127.0.0.1:5555",
    },
  },
  build: { target: "es2022", assetsDir: "blueprint_view" },
  test: {
    environment: "node",
    include: ["src/**/*.test.{ts,tsx}"],
    pool: "threads",
  },
});
