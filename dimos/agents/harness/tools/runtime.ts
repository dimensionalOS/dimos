import type { AgentTool } from "./base.ts";
import type { DetailsByTool, RuntimeToolName } from "../../shared/protocol/tools/details.ts";
import type { DimosConnection, RemoteTool } from "../dimos/connection.ts";
import type { DimosInstall } from "../dimos/install.ts";
import type { FileRenderer } from "../../shared/render/render.ts";
import type { JsonObject, ToolRegistration } from "@earendil-works/pi-durable";
import type { RuntimeToolSchemas } from "./runtime-catalog.ts";
import type { RuntimeClient } from "../dimos/client.ts";
import { DimosUnavailable } from "../dimos/connection.ts";
import { Type } from "typebox";
import { agentTool } from "./lib/tool.ts";
import { hasLiveRun } from "../dimos/install.ts";
import { streamTool } from "./lib/stream.ts";

/** Remote tools remain null until McpServer advertises them; stream_get is local. */
type RuntimeTools = { readonly [N in RuntimeToolName]: AgentTool<N> | null };

type RuntimeToolOptions = {
  readonly connection: DimosConnection;
  readonly install: DimosInstall;
  readonly renderer: FileRenderer;
  readonly runtime: RuntimeClient;
  readonly exportsDir: string;
};

type Policy<N extends RuntimeToolName> = {
  readonly replay: "safe" | "unsafe";
  readonly timeoutMs: (args: JsonObject) => number;
  readonly background: { readonly details: Omit<DetailsByTool[N], "limit" | "background"> };
};

type RemoteName = Exclude<RuntimeToolName, "stream_get">;

const POLICY: { readonly [N in RemoteName]: Policy<N> } = {
  inventory: { replay: "safe", timeoutMs: () => 30_000, background: { details: { render: null } } },
  agent_module: { replay: "unsafe", timeoutMs: () => 120_000, background: { details: { render: null } } },
};

/**
 * Remote inventory and module operations use MCP. Stream reads use the process's Web SDK session.
 */
export function runtimeTools(schemas: RuntimeToolSchemas, options: RuntimeToolOptions): RuntimeTools {
  const build = <N extends RemoteName>(name: N): AgentTool<N> | null => {
    const schema = schemas[name];
    if (schema === undefined) return null;
    const policy: Policy<N> = POLICY[name];
    return agentTool(name, registration(name, schema, policy.timeoutMs, options), {
      replay: policy.replay,
      settle: null,
      fullTextAt: null,
      wholeResult: null,
      background: policy.background,
    });
  };
  return { inventory: build("inventory"), stream_get: streamTool(options), agent_module: build("agent_module") };
}

/** What the runs directory says about whether a system exists, since the link alone cannot tell. */
function runFileNote(install: DimosInstall): string {
  return hasLiveRun(install)
    ? `A run file in ${install.runsDir} names a running DimOS system; the link reconnects on its own. Call this tool again shortly.`
    : `No run file in ${install.runsDir} names a running process, so no DimOS system is running. Start one with \`dimos run <blueprint>\` (for example \`dimos --replay run unitree-go2 --daemon\`) and call this tool again.`;
}

function registration(
  name: RemoteName,
  schema: RemoteTool,
  timeoutMs: (args: JsonObject) => number,
  options: RuntimeToolOptions,
): ToolRegistration {
  return {
    name,
    description: schema.description,
    parameters: Type.Unsafe<JsonObject>(schema.inputSchema),
    execute: async (args, api, context) => {
      const arguments_ = args as JsonObject;
      try {
        const outcome = await options.connection.callTool(name, arguments_, {
          caller: api.conversationId,
          timeoutMs: timeoutMs(arguments_),
          signal: context.abortSignal,
        });
        return {
          output: [...outcome.content],
          isError: outcome.isError,
          details: { render: null },
        };
      } catch (error) {
        if (!(error instanceof DimosUnavailable)) throw error;
        return {
          output: [{ type: "text", text: `${error.message}. ${runFileNote(options.install)}` }],
          isError: true,
          details: { render: null },
        };
      }
    },
  };
}
