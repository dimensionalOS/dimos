import assert from "node:assert/strict";
import { join } from "node:path";
import { mkdirSync, mkdtempSync, rmSync, writeFileSync } from "node:fs";
import { readModuleRecords } from "./disk.ts";
import { test } from "node:test";
import { tmpdir } from "node:os";

/** One module.json as `AgentModuleManager._save` writes it: `json.dumps(asdict(_Record), indent=2)`. */
function record(stateDir: string, name: string, fields: object): void {
  const dir = join(stateDir, "harness", "modules", name);
  mkdirSync(dir, { recursive: true });
  writeFileSync(join(dir, "module.json"), JSON.stringify(fields, null, 2));
}

test("module records are read from the state directory", async () => {
  const stateDir = mkdtempSync(join(tmpdir(), "dimos-disk-"));
  try {
    record(stateDir, "beacon", {
      owner: 2,
      class_name: "Beacon",
      notify_interval_s: 1.0,
      last_error: null,
    });
    record(stateDir, "sentinel", {
      owner: null,
      class_name: "Sentinel",
      notify_interval_s: 10.0,
      last_error: "ModuleNotFoundError: No module named 'cv3'",
    });
    mkdirSync(join(stateDir, "harness", "modules", "half_made"));
    const sources = join(stateDir, "modules", "dimos_agent");
    const reports: unknown[] = [];
    assert.deepEqual(await readModuleRecords(stateDir, (error) => reports.push(error)), [
      {
        name: "beacon",
        className: "Beacon",
        owner: 2,
        lastError: null,
        path: join(sources, "beacon.py"),
      },
      {
        name: "sentinel",
        className: "Sentinel",
        owner: null,
        lastError: "ModuleNotFoundError: No module named 'cv3'",
        path: join(sources, "sentinel.py"),
      },
    ]);
    assert.deepEqual(reports, []);
  } finally {
    rmSync(stateDir, { recursive: true, force: true });
  }
});
