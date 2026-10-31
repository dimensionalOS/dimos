/// <reference types="vitest/config" />
import deno from "@deno/vite-plugin";
import react from "@vitejs/plugin-react";
import { defineConfig, type Plugin } from "vite";

// deno() drops the subpath of dimos-react's `npm:react@^19/jsx-runtime` imports, so hand them to Vite as plain `react/...`.
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

// Relative assets under blueprint_view/ (the gateway's /blueprint_view/{file}); the dev proxy strips /gateways/dimos as Desktop does.
export default defineConfig({
  plugins: [npmReactSubpaths, deno(), react()],
  base: "./",
  resolve: { dedupe: ["react", "react-dom"] },
  server: {
    proxy: {
      "/gateways/dimos": {
        target: "http://127.0.0.1:5555",
        rewrite: (path) => path.replace(/^\/gateways\/dimos/, ""),
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
