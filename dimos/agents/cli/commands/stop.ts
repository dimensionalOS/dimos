import { stopGateway } from "#dimos/agents/cli/process.ts";
import type { Command } from "#dimos/agents/cli/commands/base.ts";

export const runStop: Command = async ({ paths }) => {
  process.stdout.write((await stopGateway(paths)) === "stopped" ? "Gateway stopped.\n" : "Gateway was not running.\n");
  return 0;
};
