import { buildInfo } from "../../gateway/config/build.ts";
import type { Command } from "./base.ts";

export const runVersion: Command = () => {
  process.stdout.write(`${JSON.stringify(buildInfo(), null, 2)}\n`);
  return 0;
};
