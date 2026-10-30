import { stopGateway } from "../process.ts";
import type { Command } from "./base.ts";

export const runStop: Command = async ({ paths }) => {
  process.stdout.write((await stopGateway(paths)) === "stopped" ? "Gateway stopped.\n" : "Gateway was not running.\n");
  return 0;
};
