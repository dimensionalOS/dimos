/// <reference types="vitest/config" />
import { fileURLToPath } from "node:url";
import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

// The cockpit imports the viewer SDK and the wire protocol straight from the
// workspace-sibling sdk/ and shared/ packages; vite needs the aliases (and
// fs.allow) because those files live outside the cockpit root. Subpath
// aliases must come first: aliases match in order and the bare one would
// otherwise swallow them.
const sdkIndex = fileURLToPath(new URL("../sdk/src/index.ts", import.meta.url));
const sdkReact = fileURLToPath(new URL("../sdk/src/react.ts", import.meta.url));
const sdkTeleop = fileURLToPath(new URL("../sdk/src/internal/teleopMachine.ts", import.meta.url));
const sharedProtocol = fileURLToPath(new URL("../shared/protocol.ts", import.meta.url));
const sharedManifest = fileURLToPath(new URL("../shared/manifest.ts", import.meta.url));
const webRoot = fileURLToPath(new URL("..", import.meta.url));

export default defineConfig({
  plugins: [react()],
  resolve: {
    alias: {
      "@dimos/sdk/internal/teleop": sdkTeleop,
      "@dimos/sdk/react": sdkReact,
      "@dimos/sdk": sdkIndex,
      "@dimos/shared/manifest": sharedManifest,
      "@dimos/shared": sharedProtocol,
    },
  },
  server: {
    // HMR dev on :5173: /api/info is answered by the relay on :7780; the
    // WebTransport connection then goes straight to the advertised wtUrl.
    proxy: { "/api": "http://127.0.0.1:7780" },
    fs: { allow: [webRoot] },
  },
  build: { target: "es2022" },
  test: {
    // UI tests (.test.tsx) opt into happy-dom per file via
    // @vitest-environment; everything else stays on node.
    environment: "node",
    include: ["src/**/*.test.{ts,tsx}"],
    // Vitest's default forks pool needs Node child-process IPC, which Deno
    // does not emulate reliably (tinypool "fd is not from BiPipe" under CI).
    pool: "threads",
  },
});
