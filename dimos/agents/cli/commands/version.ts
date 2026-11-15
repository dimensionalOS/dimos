import { buildInfo } from "#dimos/agents/gateway/config/build.ts";
import type { Command } from "#dimos/agents/cli/commands/base.ts";

export const runVersion: Command = () => {
  process.stdout.write(`${JSON.stringify(buildInfo(), null, 2)}\n`);
  return 0;
};
