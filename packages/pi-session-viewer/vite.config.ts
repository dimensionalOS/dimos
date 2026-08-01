import tailwindcss from "@tailwindcss/vite";
import react from "@vitejs/plugin-react";
import { defineConfig } from "vitest/config";

export default defineConfig({
  base: "./",
  plugins: [react(), tailwindcss()],
  build: {
    assetsDir: "assets",
    cssCodeSplit: false,
    emptyOutDir: true,
    outDir: "dist",
    sourcemap: false,
  },
  test: {
    environment: "jsdom",
    setupFiles: "./src/test/setup.ts",
  },
});
