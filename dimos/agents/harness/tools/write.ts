import type { AgentTool } from "#dimos/agents/harness/tools/base.ts";
import { agentTool } from "#dimos/agents/harness/tools/lib/tool.ts";
import { createWriteTool } from "@earendil-works/pi-durable/tools";

export function writeTool(): AgentTool<"write"> {
  return agentTool("write", createWriteTool(), {
    replay: "unsafe",
    settle: null,
    fullTextAt: null,
    wholeResult: null,
    background: null,
  });
}
