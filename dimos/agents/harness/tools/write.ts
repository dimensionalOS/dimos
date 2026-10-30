import type { AgentTool } from "./base.ts";
import { agentTool } from "./lib/tool.ts";
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
