import { parseDimosInstall } from "../../gateway/config/environment.ts";
import { runGateway } from "../../gateway/main.ts";
import type { Command } from "./base.ts";

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
