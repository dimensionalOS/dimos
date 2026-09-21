import { fileURLToPath } from "node:url";
import { defineConfig } from "vite";

// Same shape as the cockpit dev server: /api/info is answered by the relay
// on :7780 through the proxy (a convenience - the local relay also answers
// /api/* with CORS, so connect({url}) works without it), and the
// WebTransport connection then goes straight to the advertised wtUrl.
// Subpath aliases must come first: aliases match in order and the bare one
// would otherwise swallow them.
const root = fileURLToPath(new URL(".", import.meta.url));
const webRoot = fileURLToPath(new URL("../..", import.meta.url));

export default defineConfig({
  root,
  resolve: {
    alias: {
      "@dimos/sdk/react": fileURLToPath(new URL("../src/react.ts", import.meta.url)),
      "@dimos/sdk": fileURLToPath(new URL("../src/index.ts", import.meta.url)),
      "@dimos/shared/manifest": fileURLToPath(
        new URL("../../shared/manifest.ts", import.meta.url),
      ),
      "@dimos/shared": fileURLToPath(new URL("../../shared/protocol.ts", import.meta.url)),
    },
  },
  server: {
    port: 5174,
    proxy: { "/api": "http://127.0.0.1:7780" },
    fs: { allow: [webRoot] },
  },
});
