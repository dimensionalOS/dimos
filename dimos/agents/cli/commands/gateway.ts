import { parseDimosInstall } from "#dimos/agents/gateway/config/environment.ts";
import { runGateway } from "#dimos/agents/gateway/main.ts";
import type { Command } from "#dimos/agents/cli/commands/base.ts";

export const runGatewayCommand: Command = async ({ paths, env, bundledSkillsDir, daemon, startGateway }) => {
  if (daemon) {
    await startGateway();
    process.stdout.write("The gateway is running in the background. `dimos agent stop` stops it.\n");
    return 0;
  }
  await runGateway({
    paths,
    install: parseDimosInstall(env),
    bundledSkillsDir,
    env,
    providers: [],
  });
  return 0;
};
