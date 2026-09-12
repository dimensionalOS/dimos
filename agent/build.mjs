import { build } from "esbuild";
import { chmod, mkdir, rm } from "node:fs/promises";
import { resolve } from "node:path";

await rm("dist", { recursive: true, force: true });
await mkdir("dist", { recursive: true });
await build({
  entryPoints: ["src/main.ts"],
  outdir: "dist",
  bundle: true,
  platform: "node",
  target: "node24",
  format: "esm",
  packages: "external",
  sourcemap: true,
  alias: {
    "@dimos/sdk": resolve("../web/sdk/src/index.ts"),
    "@dimos/shared/manifest": resolve("../web/shared/manifest.ts"),
    "@dimos/shared": resolve("../web/shared/protocol.ts"),
  },
  banner: { js: "#!/usr/bin/env node" },
});
await chmod("dist/main.js", 0o755);
