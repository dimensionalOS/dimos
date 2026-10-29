import type { AgentTool } from "#dimos/agents/harness/tools/base.ts";
import { agentTool } from "#dimos/agents/harness/tools/lib/tool.ts";
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
