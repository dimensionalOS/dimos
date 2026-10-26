import { runTerminal } from "../../frontends/tui/main.ts";
import type { Command } from "./base.ts";

export const runTui: Command = async ({ paths, conversation, view, cwd, startGateway }) => {
  if (view && conversation === null) {
    process.stderr.write("--view needs --conversation ID.\n");
    return 2;
  }
  await startGateway();
  return runTerminal({
    socketPath: paths.socket,
    conversation,
    role: view ? "viewer" : "owner",
    cwd,
    renderCacheDir: paths.renderCacheDir,
  });
};
