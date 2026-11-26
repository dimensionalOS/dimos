import { build } from "esbuild";
import { createHash } from "node:crypto";
import { existsSync } from "node:fs";
import { chmod, mkdir, readFile, rename, rm, writeFile } from "node:fs/promises";
import { dirname, join, relative, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { webAliases } from "#dimos/agents/web-aliases.mjs";

const { metafile, outputFiles } = await build({
  entryPoints: { cli: "cli/main.ts" },
  outdir: "dist",
  write: false,
  metafile: true,
  bundle: true,
  splitting: true,
  platform: "node",
  target: "node24",
  format: "esm",
  packages: "external",
  alias: Object.fromEntries(Object.entries(webAliases).map(([name, url]) => [name, fileURLToPath(url)])),
  sourcemap: true,
  banner: { js: "#!/usr/bin/env node" },
});

// `dimos agent` rebuilds when a recorded file changed or disappeared. The record goes first and comes back last,
// so a build cut off halfway reads as stale.
await mkdir("dist", { recursive: true });
await rm("dist/inputs.json", { force: true });
// Publish the hashed chunks first so a concurrent launch can import every chunk its cli.js names.
for (const file of outputFiles) {
  await writeFile(`${file.path}.tmp`, file.contents);
  if (file.path !== resolve("dist/cli.js")) await rename(`${file.path}.tmp`, file.path);
}
await chmod("dist/cli.js.tmp", 0o755);
await rename("dist/cli.js.tmp", "dist/cli.js");

const read = new Set([
  ...Object.keys(metafile.inputs),
  ...Object.keys(metafile.outputs).filter((name) => name.endsWith(".js")),
  "build.mjs",
  "web-aliases.mjs",
  "package.json",
  "package-lock.json",
]);
// esbuild also reads the tsconfig.json nearest each input, which the metafile leaves out.
for (const input of Object.keys(metafile.inputs)) {
  for (let dir = dirname(resolve(input)); dir !== dirname(dir); dir = dirname(dir)) {
    if (existsSync(join(dir, "tsconfig.json"))) {
      read.add(relative(".", join(dir, "tsconfig.json")));
      break;
    }
  }
}
const hashes = {};
for (const name of [...read].sort())
  hashes[name] = createHash("sha256")
    .update(await readFile(name))
    .digest("hex");
await writeFile("dist/inputs.json.tmp", `${JSON.stringify(hashes, null, 2)}\n`);
await rename("dist/inputs.json.tmp", "dist/inputs.json");
