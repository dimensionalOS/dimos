import type { AgentTool } from "./base.ts";
import { agentTool } from "./lib/tool.ts";
import { createEditTool } from "@earendil-works/pi-durable/tools";

/** pi-durable's `edit`; its `{diff, patch, firstChangedLine}` details already match Pi's card. */
export function editTool(): AgentTool<"edit"> {
  return agentTool("edit", createEditTool(), {
    replay: "unsafe",
    settle: null,
    fullTextAt: null,
    wholeResult: null,
    background: null,
  });
}
