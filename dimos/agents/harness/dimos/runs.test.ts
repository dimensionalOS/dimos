import assert from "node:assert/strict";
import type { DimosRun } from "#dimos/agents/shared/protocol/runtime/runs.ts";
import { RunWatcher } from "#dimos/agents/harness/dimos/runs.ts";
import { join } from "node:path";
import { mkdtempSync, rmSync, writeFileSync } from "node:fs";
import { spawnSync } from "node:child_process";
import { test } from "node:test";
import { tmpdir } from "node:os";

function runFile(dir: string, runId: string, pid: number, startedAt: string): void {
  const entry = {
    run_id: runId,
    pid,
    blueprint: "unitree-go2",
    started_at: startedAt,
    log_dir: join(dir, "logs", runId),
    cli_args: [],
    config_overrides: {},
    original_argv: ["dimos", "run", "unitree-go2"],
  };
  writeFileSync(join(dir, `${runId}.json`), JSON.stringify(entry, null, 2));
}

test("run files are read with pid liveness, and a new file reaches listeners", async () => {
  const dir = mkdtempSync(join(tmpdir(), "dimos-runs-"));
  const exited = spawnSync("true").pid;
  assert.ok(exited !== undefined, "a process ran and exited");
  const watcher = new RunWatcher({ dir, pollMs: 60_000, report: (error) => assert.fail(String(error)) });
  try {
    runFile(dir, "20261019-120000-a1b2-unitree-go2", process.pid, "2026-10-19T12:00:00+00:00");
    runFile(dir, "20261019-110000-c3d4-unitree-go2", exited, "2026-10-19T11:00:00.250000+00:00");
    writeFileSync(join(dir, "notes.txt"), "not a run");
    watcher.start();
    assert.deepEqual(watcher.status.value, [
      {
        runId: "20261019-110000-c3d4-unitree-go2",
        pid: exited,
        blueprint: "unitree-go2",
        startedAt: 1792407600250,
        alive: false,
      },
      {
        runId: "20261019-120000-a1b2-unitree-go2",
        pid: process.pid,
        blueprint: "unitree-go2",
        startedAt: 1792411200000,
        alive: true,
      },
    ]);

    const heard = new Promise<readonly DimosRun[]>((resolve) => {
      const unsubscribe = watcher.status.subscribe((runs) => {
        if (runs.length < 3) return;
        unsubscribe();
        resolve(runs);
      });
    });
    runFile(dir, "20261019-130000-e5f6-unitree-go2", process.pid, "2026-10-19T13:00:00+00:00");
    const runs = await heard;
    assert.deepEqual(runs[2], {
      runId: "20261019-130000-e5f6-unitree-go2",
      pid: process.pid,
      blueprint: "unitree-go2",
      startedAt: 1792414800000,
      alive: true,
    });
    assert.deepEqual(
      runs.map((run) => run.runId),
      ["20261019-110000-c3d4-unitree-go2", "20261019-120000-a1b2-unitree-go2", "20261019-130000-e5f6-unitree-go2"],
    );
  } finally {
    watcher.close();
    rmSync(dir, { recursive: true, force: true });
  }
});
