// Native Pi tools generated from the live DimOS MCP catalog.
import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";
import { Type } from "@sinclair/typebox";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const HERE = dirname(fileURLToPath(import.meta.url));
const LONG_RUNNING = new Set(["move_to", "navigate_with_text", "wait", "begin_exploration", "start_patrol", "follow_person", "look_out_for"]);

export default function (pi: ExtensionAPI) {
  const base = (process.env.FRANK_URL || "http://127.0.0.1:7790").replace(/\/$/, "");
  const headers: Record<string, string> = { "Content-Type": "application/json" };
  if (process.env.FRANK_AGENT_TOKEN) headers.Authorization = `Bearer ${process.env.FRANK_AGENT_TOKEN}`;
  async function request(path: string, body?: unknown, signal?: AbortSignal) {
    const response = await fetch(`${base}${path}`, {
      method: body === undefined ? "GET" : "POST", headers,
      body: body === undefined ? undefined : JSON.stringify(body),
      signal: AbortSignal.any([AbortSignal.timeout(180000), ...(signal ? [signal] : [])]),
    });
    const payload = await response.json();
    if (!response.ok) throw new Error(JSON.stringify(payload));
    if (payload.isError) throw new Error(JSON.stringify(payload.content));
    return payload;
  }
  const catalog = JSON.parse(readFileSync(join(HERE, "..", "cache", "robot-tools.json"), "utf8"));
  for (const tool of catalog) {
    const lifecycle = LONG_RUNNING.has(tool.name)
      ? " Returns an operation_id and running status immediately. It is NOT a completed action. A tool event delivers the terminal result; operation_status and cancel_operation are optional controls."
      : "";
    pi.registerTool({
      name: tool.name,
      label: tool.name.replaceAll("_", " "),
      description: tool.description + lifecycle,
      promptSnippet: tool.description.split("\n")[0] + lifecycle,
      parameters: Type.Unsafe(tool.inputSchema),
      executionMode: "sequential",
      async execute(_id, params, signal) {
        const payload = await request("/agent/tools/call", { name: tool.name, arguments: params }, signal);
        return { content: payload.content, details: { tool: tool.name } };
      },
    });
  }
  for (const cancel of [false, true]) {
    pi.registerTool({
      name: cancel ? "cancel_operation" : "operation_status",
      label: cancel ? "Cancel operation" : "Operation status",
      description: cancel ? "Cancel a running operation by its operation_id and stop its movement." : "Inspect a running or completed operation by operation_id. Completion events arrive automatically; polling is optional.",
      parameters: Type.Object({ operation_id: Type.String() }),
      executionMode: "sequential",
      async execute(_id, params, signal) {
        const result = await request(`/agent/operations/${encodeURIComponent(params.operation_id)}${cancel ? "/cancel" : ""}`, cancel ? {} : undefined, signal);
        return { content: [{ type: "text", text: JSON.stringify(result) }], details: result };
      },
    });
  }
}
