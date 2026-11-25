import type { AgentModuleName } from "#dimos/agents/shared/protocol/runtime/event.ts";
import type { ConversationId } from "@earendil-works/pi-durable";
import { Type } from "typebox";
import { Value } from "typebox/value";
import { join } from "node:path";
import { readFile, readdir } from "node:fs/promises";

/** `$STATE/dimos/harness/modules/<name>/module.json` (`agent_module_manager.py` `_Record`): what survives DimOS being down. */
export type ModuleRecordOnDisk = {
  readonly name: AgentModuleName;
  readonly className: string;
  readonly owner: ConversationId | null;
  readonly lastError: string | null;
  /** The module's source file, which the agent edits: `$STATE/dimos/modules/dimos_agent/<name>.py`. */
  readonly path: string;
};

const ModuleJson = Type.Object({
  owner: Type.Union([Type.Integer({ minimum: 1 }), Type.Null()]),
  class_name: Type.String(),
  last_error: Type.Union([Type.String(), Type.Null()]),
});

function isMissing(error: unknown): boolean {
  return (error as NodeJS.ErrnoException).code === "ENOENT";
}

function sourceDir(stateDir: string): string {
  return join(stateDir, "modules", "dimos_agent");
}

/** Every module record HarnessRuntime left; an unreadable record is reported and skipped. */
export async function readModuleRecords(
  stateDir: string,
  report: (error: unknown) => void,
): Promise<readonly ModuleRecordOnDisk[]> {
  const root = join(stateDir, "harness", "modules");
  let names: string[];
  try {
    names = (await readdir(root, { withFileTypes: true }))
      .filter((entry) => entry.isDirectory())
      .map((entry) => entry.name)
      .sort();
  } catch (error) {
    if (!isMissing(error)) report(error);
    return [];
  }
  const records: ModuleRecordOnDisk[] = [];
  for (const name of names) {
    const file = join(root, name, "module.json");
    let json: unknown;
    try {
      json = JSON.parse(await readFile(file, "utf8"));
    } catch (error) {
      if (!isMissing(error)) report(new Error(`${file}: ${String(error)}`));
      continue;
    }
    if (!Value.Check(ModuleJson, json)) {
      const problems = [...Value.Errors(ModuleJson, json)].map(
        (problem) => `${problem.instancePath}: ${problem.message}`,
      );
      report(new Error(`${file}: ${problems.join("; ")}`));
      continue;
    }
    records.push({
      name: name as AgentModuleName,
      className: json.class_name,
      owner: json.owner === null ? null : (json.owner as ConversationId),
      lastError: json.last_error,
      path: join(sourceDir(stateDir), `${name}.py`),
    });
  }
  return records;
}
