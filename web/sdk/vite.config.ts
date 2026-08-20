import { fileURLToPath } from "node:url";
import { defineConfig } from "vite";

// Build config for the zero-build browser bundle the relay serves at
// /sdk.js (`deno task build` -> dist/sdk.js). Tests never read this file:
// vitest resolves vitest.config.ts first.
//
// Root entry only: the React bindings stay on the ./react subpath and out of
// this bundle by design. The root graph is React-free, so `external` is a
// tripwire: an accidental react import would survive as a bare specifier and
// fail loudly in the consuming browser instead of silently bundling the
// devDependency. The SDK has no dynamic imports, so the build is exactly one
// file; the relay's CORS handling relies on that (only /sdk.js gets the
// header).
const sharedProtocol = fileURLToPath(new URL("../shared/protocol.ts", import.meta.url));
const sharedManifest = fileURLToPath(new URL("../shared/manifest.ts", import.meta.url));

export default defineConfig({
  resolve: {
    // Subpath alias first: aliases match in order (same as vitest.config.ts).
    alias: {
      "@dimos/shared/manifest": sharedManifest,
      "@dimos/shared": sharedProtocol,
    },
  },
  build: {
    target: "es2022",
    lib: {
      entry: fileURLToPath(new URL("./src/index.ts", import.meta.url)),
      formats: ["es"],
      // package.json has "type": "module", so the es extension is .js and
      // this yields literally dist/sdk.js. No source maps, matching the
      // cockpit build.
      fileName: "sdk",
    },
    rollupOptions: { external: ["react", "react/jsx-runtime"] },
  },
});
