import { createHash } from "node:crypto";
import {
  cp,
  mkdir,
  readFile,
  readdir,
  rm,
  writeFile,
} from "node:fs/promises";
import { dirname, join, relative, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const packageRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const distRoot = join(packageRoot, "dist");
const targetRoot = resolve(
  packageRoot,
  "../../dimos/benchmark/spatial/pi_baseline/viewer_assets",
);

const sha256 = (value) => createHash("sha256").update(value).digest("hex");

async function filesBelow(root, current = root) {
  const entries = await readdir(current, { withFileTypes: true });
  const files = [];
  for (const entry of entries.sort((left, right) =>
    left.name.localeCompare(right.name),
  )) {
    const path = join(current, entry.name);
    if (entry.isDirectory()) {
      files.push(...(await filesBelow(root, path)));
    } else if (entry.isFile()) {
      files.push(relative(root, path).replaceAll("\\", "/"));
    }
  }
  return files;
}

await rm(targetRoot, { recursive: true, force: true });
await mkdir(targetRoot, { recursive: true, mode: 0o755 });
await cp(distRoot, targetRoot, { recursive: true });

const files = {};
for (const relativePath of await filesBelow(distRoot)) {
  files[relativePath] = sha256(await readFile(join(distRoot, relativePath)));
}
const manifest = {
  schema_version: "1.0",
  source_package: "packages/pi-session-viewer",
  package_lock_sha256: sha256(
    await readFile(join(packageRoot, "package-lock.json")),
  ),
  ai_elements_source_tag: "ai-elements@1.9.0",
  files,
};
await writeFile(
  join(targetRoot, "asset-manifest.json"),
  `${JSON.stringify(manifest, null, 2)}\n`,
  { mode: 0o644 },
);
