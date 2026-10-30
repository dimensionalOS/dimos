import type { TUI } from "@earendil-works/pi-tui";
import type { ToolCall } from "@earendil-works/pi-ai";
import type { ToolCard } from "./base.ts";
import { BashCard } from "./bash.ts";
import { PiToolCard } from "./pi-tool.ts";

/** BashCard for `bash`, PiToolCard for every other tool. */
export function cardFor(call: ToolCall, tui: TUI, cwd: string): ToolCard {
  return call.name === "bash" ? new BashCard(call, tui, cwd) : new PiToolCard(call, tui, cwd);
}
