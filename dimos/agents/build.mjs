import { build } from "esbuild";
import { createHash } from "node:crypto";
import { existsSync } from "node:fs";
import { chmod, mkdir, readFile, rename, rm, writeFile } from "node:fs/promises";
import { dirname, join, relative, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { webAliases } from "./web-aliases.mjs";

const { metafile, outputFiles } = await build({
  entryPoints: { cli: "cli/main.ts" },
  outdir: "dist",
  write: false,
  metafile: true,
  bundle: true,
  platform: "node",
  target: "node24",
  format: "esm",
  packages: "external",
  alias: Object.fromEntries(Object.entries(webAliases).map(([name, url]) => [name, fileURLToPath(url)])),
  sourcemap: true,
  banner: { js: "#!/usr/bin/env node" },
});

// `dimos agent` rebuilds when a file recorded here changed. The record goes first and comes back last,
// so a build cut off halfway reads as stale.
await mkdir("dist", { recursive: true });
await rm("dist/inputs.json", { force: true });
// A launch may run dist/cli.js meanwhile, so each output replaces the old one with an atomic rename.
for (const file of outputFiles) await writeFile(`${file.path}.tmp`, file.contents);
await chmod("dist/cli.js.tmp", 0o755);
for (const file of outputFiles) await rename(`${file.path}.tmp`, file.path);

const read = new Set([
  ...Object.keys(metafile.inputs),
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
