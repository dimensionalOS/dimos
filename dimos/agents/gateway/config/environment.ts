import type { DimosInstall } from "../../harness/dimos/install.ts";

/** Reads DIMOS_ROOT, DIMOS_PYTHON, DIMOS_CLI, DIMOS_MCP_URL, DIMOS_RUNS_DIR and DIMOS_STATE_DIR; throws naming the missing one. */
export function parseDimosInstall(env: NodeJS.ProcessEnv): DimosInstall {
  const read = (name: string): string => {
    const value = env[name];
    if (value === undefined || value === "")
      throw new Error(`${name} is not set; \`dimos agent\` sets it before starting the gateway`);
    return value;
  };
  return {
    root: read("DIMOS_ROOT"),
    python: read("DIMOS_PYTHON"),
    cli: read("DIMOS_CLI"),
    mcpUrl: new URL(read("DIMOS_MCP_URL")),
    runsDir: read("DIMOS_RUNS_DIR"),
    stateDir: read("DIMOS_STATE_DIR"),
  };
}
