import type { AgentTool, ToolPolicy } from "#dimos/agents/harness/tools/base.ts";
import type { AgentToolName } from "#dimos/agents/shared/protocol/tools/details.ts";
import type { JsonObject, ToolRegistration } from "@earendil-works/pi-durable";
import type { TSchema } from "typebox";
import { Type } from "typebox";

export const MAX_CHARS = "max_chars";

const maxCharsSchema = Type.Optional(
  Type.Integer({
    minimum: 1,
    description:
      "How many characters of the result to show; default 2000, clamped to the context left. The whole result is saved to a file.",
  }),
);

/** The tool's parameters with `max_chars` spliced in; a TypeBox object and an MCP JSON Schema object are both JSON Schema. */
function withMaxChars(parameters: TSchema): TSchema {
  const properties =
    "properties" in parameters && typeof parameters["properties"] === "object" ? parameters["properties"] : {};
  return Type.Unsafe<unknown>({ ...parameters, properties: { ...properties, [MAX_CHARS]: maxCharsSchema } });
}

/**
 * Adds the optional integer `max_chars` to the tool's parameters, removes it before `execute`, and sets `replay`.
 * Throws when `tool.name` is not `name`. afterTool's `call.arguments` still holds the model's `max_chars`, as
 * harness/main.test.ts shows.
 */
export function agentTool<N extends AgentToolName>(
  name: N,
  tool: ToolRegistration,
  policy: ToolPolicy<N>,
): AgentTool<N> {
  if (tool.name !== name)
    throw new Error(`Tool ${JSON.stringify(tool.name)} cannot be registered as ${JSON.stringify(name)}`);
  const registration: ToolRegistration = {
    ...tool,
    parameters: withMaxChars(tool.parameters),
    replay: policy.replay,
    execute: (args, api, context) => {
      const { [MAX_CHARS]: _maxChars, ...rest } = args as JsonObject;
      return tool.execute(rest, api, context);
    },
  };
  return { ...policy, name, registration };
}
