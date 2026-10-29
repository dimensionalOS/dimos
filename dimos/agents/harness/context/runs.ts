import type { Fact, FactKey, Facts } from "#dimos/agents/shared/protocol/runtime/context.ts";
import { Type } from "typebox";
import { Value } from "typebox/value";
import { isAlive } from "#dimos/agents/harness/dimos/runs.ts";
import { join } from "node:path";
import { readFile, readdir } from "node:fs/promises";

/** The fields of DimOS's run registry file (dimos/core/run_registry.py `RunEntry`) the gateway reads. */
const RunFile = Type.Object({ run_id: Type.String(), pid: Type.Integer({ minimum: 1 }), blueprint: Type.String() });

/**
 * One `run:<run id>` fact per running system: each file in DimOS's runs directory, which `dimos run` writes per system
 * and removes when it stops, whose pid is alive. A file left by a run that exited uncleanly is skipped, and so is one
 * that does not parse, such as a file still being written.
 */
export async function runFacts(runsDir: string): Promise<Facts> {
  const names = await readdir(runsDir).catch(() => []);
  const facts: { [key: FactKey]: Fact } = {};
  for (const name of names.filter((entry) => entry.endsWith(".json")).sort()) {
    let json: unknown;
    try {
      json = JSON.parse(await readFile(join(runsDir, name), "utf8"));
    } catch {
      continue;
    }
    if (!Value.Check(RunFile, json) || !isAlive(json.pid)) continue;
    facts[`run:${json.run_id}`] = { section: "Environment", line: `Run ${json.run_id} (blueprint ${json.blueprint})` };
  }
  return facts;
}
