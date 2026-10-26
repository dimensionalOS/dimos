import type { AgentToolName, Background, RuntimeToolName, ToolDetails } from "../../shared/protocol/tools/details.ts";
import { DETAILS, MovedSchema } from "../../shared/protocol/tools/details.ts";
import { Value } from "typebox/value";

/** The `background` of a result whose work moved to the background, read before any settle step; null otherwise. */
export function backgroundOf(details: unknown): Background | null {
  return Value.Check(MovedSchema, details) ? details.background : null;
}

export function isAgentToolName(name: string): name is AgentToolName {
  return Object.hasOwn(DETAILS, name);
}

export function isRuntimeToolName(name: string): name is RuntimeToolName {
  return name === "inventory" || name === "stream_get" || name === "agent_module";
}

/** Checks stored `details` against the schema of the tool named on the result. Only the gateway's view calls it. */
export function parseToolDetails(toolName: string, details: unknown): ToolDetails {
  if (!isAgentToolName(toolName) || !Value.Check(DETAILS[toolName], details)) return { tool: null };
  return { tool: toolName, details } as ToolDetails;
}
