import type { AgentTool } from "#dimos/agents/harness/tools/base.ts";
import type { Detacher } from "#dimos/agents/harness/background/detacher.ts";
import type { MovedCall } from "#dimos/agents/shared/protocol/conversation/calls.ts";
import type { ToolExecutionResult } from "@earendil-works/pi-durable";
import { Type } from "typebox";
import { agentTool } from "#dimos/agents/harness/tools/lib/tool.ts";
import { defineTool } from "@earendil-works/pi-durable";

const BackgroundParameters = Type.Object({
  op: Type.Union([Type.Literal("list"), Type.Literal("output"), Type.Literal("stop")]),
  call: Type.Optional(
    Type.String({ description: "The moved call's id, as its result and the list name it. output and stop need it." }),
  ),
});

function text(value: string, isError = false): ToolExecutionResult {
  return { output: [{ type: "text", text: value }], isError, details: {} };
}

function describe(moved: MovedCall, now: number): string {
  const seconds = Math.round((now - moved.since) / 1000);
  return `${moved.call}: ${moved.tool} ${JSON.stringify(moved.arguments)}, running for ${seconds} s`;
}

/** The `background` tool over the Detacher, for this conversation's moved calls; the gateway's `stopBackground` stops them alike. */
export function backgroundTool(detacher: Detacher): AgentTool<"background"> {
  const tool = defineTool({
    name: "background",
    description:
      "List this conversation's tool calls that run in the background, read one's output so far, or stop one. A call moves to the background when the user sends it there. Its result arrives later as a message, and so does the output of a call that is stopped.",
    parameters: BackgroundParameters,
    execute: async (args, api) => {
      const moved = detacher.moved(api.conversationId).value;
      if (args.op === "list") {
        if (moved.length === 0) return text("No calls run in the background.");
        const now = Date.now();
        return text(moved.map((call) => describe(call, now)).join("\n"));
      }
      const target = moved.find((call) => call.call === args.call);
      if (target === undefined) return text(`No call ${JSON.stringify(args.call ?? "")} runs in the background.`, true);
      if (args.op === "output") {
        if (target.output === "") return text(`${target.call} has written no output yet.`);
        return text(target.truncated ? `[earlier output was cut]\n${target.output}` : target.output);
      }
      return detacher.stop(api.conversationId, target.call)
        ? text(`Stopped ${target.call}. Its output until now arrives as a message.`)
        : text(`${target.call} ended before it could be stopped; its result arrives as a message.`, true);
    },
  });
  return agentTool("background", tool, {
    replay: "unsafe",
    settle: null,
    fullTextAt: null,
    wholeResult: null,
    background: null,
  });
}
