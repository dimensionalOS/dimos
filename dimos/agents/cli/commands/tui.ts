import { runTerminal } from "../../frontends/tui/main.ts";
import type { Command } from "./base.ts";

export const runTui: Command = ({ paths, conversation, view, cwd }) =>
  runTerminal({
    socketPath: paths.socket,
    conversation,
    role: view ? "viewer" : "owner",
    cwd,
    renderCacheDir: paths.renderCacheDir,
  });
