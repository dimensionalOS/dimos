import { fileURLToPath } from "node:url";
import { defineConfig } from "vitest/config";

// The SDK imports the wire protocol from the workspace-sibling shared/
// package; vitest needs the aliases because those files live outside this
// package root. The subpath alias must come first: aliases match in order
// and the bare one would otherwise swallow it.
const sharedProtocol = fileURLToPath(new URL("../shared/protocol.ts", import.meta.url));
const sharedManifest = fileURLToPath(new URL("../shared/manifest.ts", import.meta.url));

export default defineConfig({
  // react.test.tsx uses JSX without @vitejs/plugin-react; the automatic
  // runtime resolves react/jsx-runtime from the workspace node_modules.
  esbuild: { jsx: "automatic" },
  resolve: {
    alias: {
      "@dimos/shared/manifest": sharedManifest,
      "@dimos/shared": sharedProtocol,
    },
  },
  test: {
    // React hook tests (.test.tsx) opt into happy-dom per file via
    // @vitest-environment; everything else stays on node.
    environment: "node",
    include: ["src/**/*.test.{ts,tsx}"],
    // Vitest's default forks pool needs Node child-process IPC, which Deno
    // does not emulate reliably (tinypool "fd is not from BiPipe" under CI).
    pool: "threads",
  },
});
