import { execFileSync } from "node:child_process";
import type { OpenClawPluginApi } from "openclaw/plugin-sdk";
import { Type } from "@sinclair/typebox";
import type { AnyAgentTool } from "openclaw/agents/tools/common.js";

const DEFAULT_HOST = "127.0.0.1";
const DEFAULT_PORT = 9990;
const CALL_TIMEOUT_MS = 30_000;

interface McpToolDef {
  name: string;
  description: string;
  inputSchema: Record<string, unknown>;
}

function getHost(pluginConfig?: Record<string, unknown>): string {
  if (pluginConfig && typeof pluginConfig.mcpHost === "string" && pluginConfig.mcpHost) {
    return pluginConfig.mcpHost;
  }
  return DEFAULT_HOST;
}

function getPort(pluginConfig?: Record<string, unknown>): number {
  if (pluginConfig && typeof pluginConfig.mcpPort === "number") {
    return pluginConfig.mcpPort;
  }
  return DEFAULT_PORT;
}

function mcpUrl(host: string, port: number): string {
  return `http://${host}:${port}/mcp`;
}

/** Send a JSON-RPC request to the DimOS MCP HTTP server. */
async function rpc(
  url: string,
  method: string,
  params?: Record<string, unknown>,
  timeoutMs: number = CALL_TIMEOUT_MS,
): Promise<unknown> {
  const body = {
    jsonrpc: "2.0",
    id: 1,
    method,
    ...(params ? { params } : {}),
  };
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs);
  try {
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
      signal: controller.signal,
    });
    if (!res.ok) {
      throw new Error(`HTTP ${res.status}: ${await res.text()}`);
    }
    const json = (await res.json()) as { result?: unknown; error?: { message: string } };
    if (json.error) {
      throw new Error(json.error.message);
    }
    return json.result;
  } finally {
    clearTimeout(timer);
  }
}

/** Convert a JSON Schema properties object into a TypeBox Type.Object schema. */
function jsonSchemaToTypebox(
  inputSchema?: Record<string, unknown>,
): ReturnType<typeof Type.Object> {
  if (!inputSchema) {
    return Type.Object({});
  }

  const properties = (inputSchema.properties ?? {}) as Record<string, Record<string, unknown>>;
  const required = new Set((inputSchema.required ?? []) as string[]);
  const tbProps: Record<string, unknown> = {};

  for (const [key, prop] of Object.entries(properties)) {
    const desc = typeof prop.description === "string" ? prop.description : undefined;
    let inner;
    switch (prop.type) {
      case "number":
      case "integer":
        inner = Type.Number({ description: desc });
        break;
      case "boolean":
        inner = Type.Boolean({ description: desc });
        break;
      case "array":
        inner = Type.Array(Type.Unknown(), { description: desc });
        break;
      case "object":
        inner = Type.Record(Type.String(), Type.Unknown(), { description: desc });
        break;
      default:
        inner = Type.String({ description: desc });
        break;
    }
    tbProps[key] = required.has(key) ? inner : Type.Optional(inner);
  }

  // @ts-expect-error -- heterogeneous property map built dynamically
  return Type.Object(tbProps);
}

/**
 * Discover MCP tools synchronously by sending HTTP requests to the DimOS MCP server.
 * Uses a child process so we can block the main thread during plugin registration.
 */
function discoverToolsSync(host: string, port: number): McpToolDef[] {
  const url = mcpUrl(host, port);
  const script = `
const http = require('http');
function post(body) {
  return new Promise((resolve, reject) => {
    const data = JSON.stringify(body);
    const req = http.request(${JSON.stringify(url)}, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(data) },
    }, (res) => {
      let buf = '';
      res.on('data', d => buf += d);
      res.on('end', () => {
        if (res.statusCode !== 200) { reject(new Error('HTTP ' + res.statusCode + ': ' + buf)); return; }
        resolve(JSON.parse(buf));
      });
    });
    req.on('error', reject);
    req.write(data);
    req.end();
  });
}
(async () => {
  await post({jsonrpc:'2.0',id:1,method:'initialize',params:{protocolVersion:'2024-11-05',capabilities:{},clientInfo:{name:'openclaw-dimos',version:'0.0.1'}}});
  const res = await post({jsonrpc:'2.0',id:2,method:'tools/list',params:{}});
  if (!res.result || !Array.isArray(res.result.tools)) { process.stderr.write('Unexpected response: ' + JSON.stringify(res)); process.exit(1); }
  process.stdout.write(JSON.stringify(res.result.tools));
})().catch(e => { process.stderr.write(e.message); process.exit(1); });
`;
  const result = execFileSync("node", ["-e", script], {
    timeout: 10_000,
    encoding: "utf-8",
  });
  return JSON.parse(result);
}

/** Track whether we have sent an initialize handshake from the main process. */
let initialized = false;

/** Call an MCP tool via HTTP POST to the DimOS server. */
async function callTool(
  host: string,
  port: number,
  name: string,
  args: Record<string, unknown>,
): Promise<string> {
  const url = mcpUrl(host, port);
  if (!initialized) {
    await rpc(url, "initialize", {
      protocolVersion: "2024-11-05",
      capabilities: {},
      clientInfo: { name: "openclaw-dimos", version: "0.0.1" },
    });
    initialized = true;
  }
  const result = (await rpc(url, "tools/call", { name, arguments: args })) as {
    content?: Array<{ type: string; text?: string }>;
  };
  const content = result?.content;
  if (Array.isArray(content)) {
    return (
      content
        .filter((c) => c.type === "text")
        .map((c) => c.text)
        .join("\n") || "No content returned"
    );
  }
  return content != null ? JSON.stringify(content) : "No content returned";
}

export default {
  id: "dimos",
  name: "Dimos MCP Bridge",
  description: "Exposes tools from the dimos MCP server as OpenClaw agent tools",

  register(api: OpenClawPluginApi) {
    const host = getHost(api.pluginConfig);
    const port = getPort(api.pluginConfig);

    let mcpTools: McpToolDef[];
    try {
      mcpTools = discoverToolsSync(host, port);
      api.logger.info(`dimos: discovered ${mcpTools.length} tool(s) from ${host}:${port}`);
    } catch (err) {
      api.logger.error(`dimos: failed to discover tools from ${host}:${port}: ${err}`);
      return;
    }

    for (const mcpTool of mcpTools) {
      const parameters = jsonSchemaToTypebox(mcpTool.inputSchema);
      const tool: AnyAgentTool = {
        name: mcpTool.name,
        label: mcpTool.name,
        description: mcpTool.description || "",
        parameters,
        async execute(
          _toolCallId: string,
          params: Record<string, unknown>,
        ): Promise<{ content: Array<{ type: "text"; text: string }>; details: unknown }> {
          const text = await callTool(host, port, mcpTool.name, params);
          return {
            content: [{ type: "text" as const, text }],
            details: { tool: mcpTool.name, params },
          };
        },
      };
      api.registerTool(tool, { name: mcpTool.name });
    }
  },
};
