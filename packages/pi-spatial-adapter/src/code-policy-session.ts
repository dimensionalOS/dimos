import { Type } from "typebox";
import type { ToolDefinition } from "@earendil-works/pi-coding-agent";
import {
  createFreshSessionWithTools,
  type SessionAdapterHandle,
  type SessionConfig,
  type StoredAuthOptions,
} from "./session.js";

export const CODE_POLICY_TOOL_NAME = "python_exec" as const;
export const CODE_POLICY_TOOL_NAMES = [CODE_POLICY_TOOL_NAME] as const;

export interface CodePolicyBroker {
  request(
    tool: typeof CODE_POLICY_TOOL_NAME,
    params: { code: string; timeout_s?: number },
  ): Promise<string>;
}

export function codePolicyToolDefinition(broker: CodePolicyBroker): ToolDefinition {
  return {
    name: CODE_POLICY_TOOL_NAME,
    label: "Execute Python",
    description:
      "Execute one synchronous Python program in the persistent trusted, unsandboxed DimOS policy session. The session preloads app for deployed DimOS RPCs and memory for observations.",
    parameters: Type.Object(
      {
        code: Type.String({ minLength: 1 }),
        timeout_s: Type.Optional(Type.Number({ exclusiveMinimum: 0, maximum: 110 })),
      },
      { additionalProperties: false },
    ),
    execute: async (_id, params) => ({
      content: [
        {
          type: "text",
          text: await broker.request(
            CODE_POLICY_TOOL_NAME,
            params as { code: string; timeout_s?: number },
          ),
        },
      ],
      details: {},
    }),
  };
}

export async function createFreshCodePolicySession(
  broker: CodePolicyBroker,
  options: StoredAuthOptions,
  config: SessionConfig,
  initialPrompt: string,
): Promise<SessionAdapterHandle> {
  const result = await createFreshSessionWithTools(
    [codePolicyToolDefinition(broker)],
    options,
    config,
    initialPrompt,
    CODE_POLICY_TOOL_NAMES,
  );
  if (
    result.activeToolNames.length !== 1 ||
    result.activeToolNames[0] !== CODE_POLICY_TOOL_NAME
  ) {
    result.handle.dispose();
    throw new Error("Pi code-policy session did not activate exactly python_exec");
  }
  return result.handle;
}
