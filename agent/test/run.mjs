import { build } from "esbuild";
import { readdir } from "node:fs/promises";
import { resolve } from "node:path";
import { spawn } from "node:child_process";
const tests = (await readdir("test")).filter((name) =>
  name.endsWith(".test.ts"),
);
await build({
  entryPoints: [...tests.map((name) => "test/" + name), "src/main.ts"],
  outbase: ".",
  outdir: ".test",
  bundle: true,
  format: "esm",
  platform: "node",
  target: "node24",
  packages: "external",
  alias: {
    "@dimos/sdk": resolve("../web/sdk/src/index.ts"),
    "@dimos/shared/manifest": resolve("../web/shared/manifest.ts"),
    "@dimos/shared": resolve("../web/shared/protocol.ts"),
  },
});
const child = spawn(
  process.execPath,
  [
    "--test",
    ...tests.map((name) => ".test/test/" + name.replace(/\.ts$/, ".js")),
  ],
  { stdio: "inherit" },
);
child.on("exit", (code) => {
  process.exitCode = code ?? 1;
});
