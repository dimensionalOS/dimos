import { runTerminal } from "#dimos/agents/frontends/tui/main.ts";
import type { Command } from "#dimos/agents/cli/commands/base.ts";

export const runTui: Command = ({ paths, conversation, view, cwd, profile }) =>
  runTerminal({
    socketPath: paths.socket,
    conversation,
    role: view ? "viewer" : "owner",
    cwd,
    profile,
    renderCacheDir: paths.renderCacheDir,
  });
