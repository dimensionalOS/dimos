import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";
import { Type } from "typebox";

const DEFAULT_MCP_ENDPOINT = "http://127.0.0.1:9990/mcp";
const DEFAULT_TIMEOUT_S = 110;

interface JsonRpcResponse {
  readonly result?: unknown;
  readonly error?: {
    readonly code?: number;
    readonly message?: string;
  };
}

function endpoint(): string {
  return process.env.PI_SPATIAL_MCP_ENDPOINT ?? DEFAULT_MCP_ENDPOINT;
}

async function mcpRequest(
  method: string,
  params: Record<string, unknown> | undefined,
  timeoutS: number,
  signal?: AbortSignal,
): Promise<unknown> {
  const timeoutSignal = AbortSignal.timeout(timeoutS * 1_000);
  const response = await fetch(endpoint(), {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({
      jsonrpc: "2.0",
      id: crypto.randomUUID(),
      method,
      ...(params ? { params } : {}),
    }),
    signal: signal ? AbortSignal.any([signal, timeoutSignal]) : timeoutSignal,
  });
  if (!response.ok) {
    throw new Error(`MCP request failed with HTTP ${response.status}`);
  }
  const payload = (await response.json()) as JsonRpcResponse;
  if (payload.error) {
    throw new Error(
      `MCP error ${payload.error.code ?? "unknown"}: ${payload.error.message ?? "unknown error"}`,
    );
  }
  return payload.result;
}

export function inspectManualToolInventory(result: unknown): void {
  if (typeof result !== "object" || result === null || Array.isArray(result)) {
    throw new Error("MCP tools/list returned an invalid result");
  }
  const tools = (result as { tools?: unknown }).tools;
  if (!Array.isArray(tools)) throw new Error("MCP tools/list omitted its tool inventory");
  const matches = tools.filter(
    (tool) =>
      typeof tool === "object" &&
      tool !== null &&
      !Array.isArray(tool) &&
      (tool as { name?: unknown }).name === "python_exec",
  );
  if (matches.length !== 1) {
    throw new Error(
      "the attached DimOS stack must expose exactly one python_exec skill",
    );
  }
}

export function textFromToolResult(result: unknown): string {
  if (typeof result !== "object" || result === null || Array.isArray(result)) {
    throw new Error("MCP tools/call returned an invalid result");
  }
  const content = (result as { content?: unknown }).content;
  if (!Array.isArray(content)) throw new Error("MCP tools/call omitted content");
  return content
    .map((item) => {
      if (typeof item !== "object" || item === null || Array.isArray(item)) {
        return JSON.stringify(item);
      }
      const text = (item as { text?: unknown }).text;
      return typeof text === "string" ? text : JSON.stringify(item);
    })
    .join("\n");
}

export default async function manualCodePolicyExtension(
  pi: ExtensionAPI,
): Promise<void> {
  inspectManualToolInventory(await mcpRequest("tools/list", undefined, 10));

  pi.registerTool({
    name: "python_exec",
    label: "Execute Python",
    description:
      "Execute one synchronous Python program in the persistent trusted, unsandboxed DimOS policy session. The session preloads app for deployed DimOS RPCs and memory for observations.",
    parameters: Type.Object(
      {
        code: Type.String({ minLength: 1 }),
        timeout_s: Type.Optional(
          Type.Number({ exclusiveMinimum: 0, maximum: DEFAULT_TIMEOUT_S }),
        ),
      },
      { additionalProperties: false },
    ),
    execute: async (_toolCallId, params, signal) => {
      const input = params as { code: string; timeout_s?: number };
      const timeoutS = input.timeout_s ?? DEFAULT_TIMEOUT_S;
      const result = await mcpRequest(
        "tools/call",
        {
          name: "python_exec",
          arguments: { code: input.code, timeout_s: timeoutS },
        },
        timeoutS + 5,
        signal,
      );
      return {
        content: [{ type: "text", text: textFromToolResult(result) }],
        details: {},
      };
    },
  });
}
