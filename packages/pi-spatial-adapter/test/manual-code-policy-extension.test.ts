import assert from "node:assert/strict";
import test from "node:test";
import {
  inspectManualToolInventory,
  textFromToolResult,
} from "../src/manual-code-policy-extension.js";

test("manual inventory accepts one python_exec among host-only MCP tools", () => {
  assert.doesNotThrow(() =>
    inspectManualToolInventory({
      tools: [{ name: "move" }, { name: "python_exec" }, { name: "server_status" }],
    }),
  );
});

test("manual inventory rejects missing and duplicate python_exec tools", () => {
  assert.throws(
    () => inspectManualToolInventory({ tools: [{ name: "move" }] }),
    /exactly one python_exec/,
  );
  assert.throws(
    () =>
      inspectManualToolInventory({
        tools: [{ name: "python_exec" }, { name: "python_exec" }],
      }),
    /exactly one python_exec/,
  );
});

test("manual tool result renders all MCP text content", () => {
  assert.equal(
    textFromToolResult({
      content: [{ type: "text", text: "first" }, { type: "text", text: "second" }],
    }),
    "first\nsecond",
  );
});
