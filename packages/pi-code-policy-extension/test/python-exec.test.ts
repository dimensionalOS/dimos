import assert from "node:assert/strict";
import test from "node:test";

import type { CallToolResult } from "@modelcontextprotocol/client";
import type { ExtensionAPI, ToolDefinition } from "@earendil-works/pi-coding-agent";

import { installPythonExec } from "../src/python-exec.js";

test("registers one tool that calls MCP directly", async () => {
  let tool: ToolDefinition | undefined;
  let shutdown: (() => Promise<void>) | undefined;
  let closed = false;
  const pi = {
    registerTool(value: ToolDefinition) {
      tool = value;
    },
    on(event: string, handler: () => Promise<void>) {
      if (event === "session_shutdown") shutdown = handler;
    },
  } as ExtensionAPI;
  const client = {
    async listTools() {
      return {
        tools: [
          {
            name: "python_exec",
            description: "Canonical CodePolicy description",
          },
        ],
      };
    },
    async callTool(params: { name: string; arguments: Record<string, unknown> }) {
      assert.deepEqual(params, {
        name: "python_exec",
        arguments: { code: "1 + 1", timeout_s: 3 },
      });
      return { content: [{ type: "text", text: "2" }] } as CallToolResult;
    },
    async close() {
      closed = true;
    },
  };

  await installPythonExec(pi, "http://127.0.0.1:1/mcp", async () => client);
  assert.equal(tool?.name, "python_exec");
  assert.equal(tool?.description, "Canonical CodePolicy description");
  const result = await tool!.execute("call-1", { code: "1 + 1", timeout_s: 3 }, undefined, undefined, {} as never);
  assert.deepEqual(result.content, [{ type: "text", text: "2" }]);
  await shutdown!();
  assert.equal(closed, true);
});
