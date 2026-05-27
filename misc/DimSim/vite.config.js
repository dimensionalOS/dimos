import { defineConfig } from "vite";

export default defineConfig({
  optimizeDeps: {
    exclude: ["@dimforge/rapier3d-compat"],
  },
  assetsInclude: ["**/*.wasm"],
  build: {
    assetsInlineLimit: 0,
    rollupOptions: {
      external: [/^https:\/\/esm\.sh\//],
      output: {
        // Eval workflow files under scenes/*/evals/*.js import runEval from
        // '@dimsim/eval' via the importmap in index.html.  That map needs to
        // point at a *stable* URL, so pin the harness chunk's filename here
        // (it's the public ESM surface for evals).  Everything else keeps
        // its content-hashed name.
        chunkFileNames(chunk) {
          if (chunk.facadeModuleId?.endsWith("/evals/harness.ts")) {
            return "assets/dimsim-eval.js";
          }
          return "assets/[name]-[hash].js";
        },
      },
    },
  },
  server: {
    proxy: {
      "/vlm": {
        target: "http://127.0.0.1:8000",
        changeOrigin: true,
      },
    },
  },
});
