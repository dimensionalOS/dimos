import assert from "node:assert/strict";
import test from "node:test";
import {
  CODE_POLICY_TOOL_NAMES,
  codePolicyToolDefinition,
} from "../src/code-policy-session.js";

test("code-policy facade registers exactly python_exec", async () => {
  const calls: Array<{ tool: string; params: unknown }> = [];
  const tool = codePolicyToolDefinition({
    request: async (name, params) => {
      calls.push({ tool: name, params });
      return "ok";
    },
  });

  assert.deepEqual(CODE_POLICY_TOOL_NAMES, ["python_exec"]);
  assert.equal(tool.name, "python_exec");
  const result = await tool.execute(
    "call-1",
    { code: "1 + 1" },
    undefined,
    undefined,
    undefined as never,
  );
  assert.deepEqual(calls, [
    { tool: "python_exec", params: { code: "1 + 1" } },
  ]);
  assert.deepEqual(result.content, [{ type: "text", text: "ok" }]);
});
