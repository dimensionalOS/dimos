import type { AgentTool } from "./base.ts";
import type { AgentToolName } from "../../shared/protocol/tools/details.ts";
import type { Context } from "@earendil-works/chord/context";
import type { Detacher } from "../background/detacher.ts";
import type { Extension, JsonObject, ToolExecutionResult, ToolHookCall } from "@earendil-works/pi-durable";
import type { ImageContent, TextContent } from "@earendil-works/pi-ai";
import type { OutputLimiter } from "../limit/limit.ts";
import { MAX_CHARS } from "./lib/tool.ts";
import { ToolTask, defineExtension, hook } from "@earendil-works/pi-durable";
import { backgroundOf, isAgentToolName } from "./details.ts";

/** One slot per tool of the plan's table, so a missing or extra tool fails to compile. Null: not offered now. */
export type AgentToolTable = { readonly [N in AgentToolName]: AgentTool<N> | null };

function lookup(table: AgentToolTable, name: string): AgentTool<AgentToolName> | null {
  return isAgentToolName(name) ? table[name] : null;
}

function textOf(output: readonly (TextContent | ImageContent)[]): string {
  return output.flatMap((block) => (block.type === "text" ? [block.text] : [])).join("\n");
}

function integer(value: unknown): number | null {
  return typeof value === "number" && Number.isInteger(value) && value >= 1 ? value : null;
}

function detailsObject(details: unknown): JsonObject {
  return typeof details === "object" && details !== null && !Array.isArray(details) ? (details as JsonObject) : {};
}

/** A result whose work moved to the background has nothing to settle yet; it gets the tool's details for that case. */
async function settle(
  tool: AgentTool<AgentToolName>,
  result: ToolExecutionResult,
  call: ToolHookCall,
  context: Context,
): Promise<ToolExecutionResult> {
  const moved = backgroundOf(result.details);
  if (tool.background !== null && moved !== null)
    return { ...result, details: { ...tool.background.details, background: moved } };
  return tool.settle === null ? result : tool.settle(result, call, context);
}

/**
 * The agent's tools, those whose policy allows it wrapped so a running call can move to the background, and the
 * afterTool hook every result passes: the tool's settle step, then the `max_chars` cut.
 */
export function toolsExtension(table: AgentToolTable, limiter: OutputLimiter, detacher: Detacher): Extension {
  return defineExtension({
    name: "dimos-tools",
    tools: Object.values(table).flatMap((tool) =>
      tool === null ? [] : [tool.background === null ? tool.registration : detacher.wrap(tool)],
    ),
    hooks: [
      hook(ToolTask, {
        afterTool: async (call, result, api, context) => {
          const tool = lookup(table, call.name);
          if (tool === null) return undefined;
          const settled = await settle(tool, result, call, context);
          const output = settled.output ?? [];
          const text = textOf(output);
          const requested = integer(call.arguments[MAX_CHARS]);
          const limited = await limiter.limit(text, {
            conversation: api.conversationId,
            requested: requested ?? (tool.wholeResult !== null && tool.wholeResult(call) ? text.length : null),
            spillName: call.id,
            fullTextAt: tool.fullTextAt === null ? null : tool.fullTextAt(settled, call),
            argument: MAX_CHARS,
          });
          return {
            ...settled,
            output:
              limited.record === null
                ? output
                : [{ type: "text", text: limited.text }, ...output.filter((block) => block.type === "image")],
            details: { ...detailsObject(settled.details), limit: limited.record },
          };
        },
      }),
    ],
  });
}

/** Every tool not declared replay-safe may change the running system; the runtime context observes after those. */
export function mayChangeSystem(table: AgentToolTable): (toolName: string) => boolean {
  return (toolName) => {
    const tool = lookup(table, toolName);
    return tool !== null && tool.replay === "unsafe";
  };
}
