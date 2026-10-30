import type { BuildInfo } from "../../shared/protocol/lifecycle/contract.ts";
import { createHash } from "node:crypto";
import { fileURLToPath } from "node:url";
import { existsSync, readFileSync, realpathSync, statSync } from "node:fs";
import { resolve } from "node:path";

function packageVersion(): string {
  const bundled = new URL("../package.json", import.meta.url);
  const source = new URL("../../package.json", import.meta.url);
  const json: unknown = JSON.parse(readFileSync(existsSync(bundled) ? bundled : source, "utf8"));
  if (typeof json !== "object" || json === null || !("version" in json) || typeof json.version !== "string")
    return "unversioned";
  return json.version;
}

/** The script node runs, or this module's own file when node runs none (`node -e`, a REPL). */
function runningScript(): string {
  const script = process.argv[1];
  if (script !== undefined && statSync(script, { throwIfNoEntry: false })?.isFile())
    return realpathSync(resolve(script));
  return fileURLToPath(import.meta.url);
}

const executable = runningScript();

// Captured at load, before an upgrade can replace the file beneath a running gateway.
const loaded: BuildInfo = {
  version: packageVersion(),
  build: createHash("sha256").update(readFileSync(executable)).digest("hex").slice(0, 12),
  executable,
  node: process.version,
};

export function buildInfo(): BuildInfo {
  return loaded;
}
