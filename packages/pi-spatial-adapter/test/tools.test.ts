import test from "node:test";
import assert from "node:assert/strict";
import { TOOL_NAMES, assertNoBuiltinTools, assertToolInventory, submitAnswerParameters, toolDefinitions, createBroker } from "../src/tools.js";

test("accepts exactly the three host-supplied tools", () => {
  assert.doesNotThrow(() => assertToolInventory(TOOL_NAMES));
  assert.throws(() => assertToolInventory(["sandbox_exec", "render_case"]));
  assert.throws(() => assertToolInventory(["sandbox_exec", "render_case", "submit_answer", "bash"]));
  assert.doesNotThrow(() => assertNoBuiltinTools(TOOL_NAMES));
  assert.throws(() => assertNoBuiltinTools(["read"]));
});

test("uses workspace image paths and public answer-type schemas", () => {
  const broker = createBroker(() => undefined, 1);
  const booleanSubmit = toolDefinitions(broker, "boolean").find((tool) => tool.name === "submit_answer");
  const integerSubmit = toolDefinitions(broker, "integer").find((tool) => tool.name === "submit_answer");
  assert.deepEqual(booleanSubmit?.parameters, submitAnswerParameters("boolean"));
  assert.deepEqual(integerSubmit?.parameters, submitAnswerParameters("integer"));
  const image = toolDefinitions(broker, "boolean").find((tool) => tool.name === "read_generated_image");
  assert.match(JSON.stringify(image?.parameters), /"path"/);
  assert.match(JSON.stringify(image?.parameters), /512/);
  assert.equal(JSON.stringify(image?.parameters).includes("\\\\.\\\\."), true);
});

test("encouraged image success appends the static stop acknowledgement after the image", async () => {
  const frames: Array<Record<string, unknown>> = [];
  const broker = createBroker((frame) => frames.push(frame as Record<string, unknown>), 1);
  const imageTool = toolDefinitions(broker, "boolean", "visualization_encouraged").find(
    (tool) => tool.name === "read_generated_image",
  );
  assert.ok(imageTool);
  const pending = imageTool.execute("tool-1", { path: "image.png" }, undefined, undefined, undefined as never);
  broker.reply({
    version: 2,
    type: "tool_reply",
    id: "tool-1",
    ok: true,
    result: { mime: "image/png", data: "iVBORw0KGgo=" },
  });
  const result = await pending;
  assert.equal(result.content.length, 2);
  assert.equal(result.content[0].type, "image");
  assert.equal(result.content[1].type, "text");
  assert.equal(
    result.content[1].text,
    "Visualization requirement satisfied. Stop further analysis now. Call submit_answer exactly once with your best answer from the evidence already available, even if uncertain. Do not call sandbox_exec or read_generated_image again.",
  );
  assert.equal(frames.length, 1);
});

test("forbidden image success and image errors do not receive acknowledgement", async () => {
  const forbiddenBroker = createBroker(() => undefined, 1);
  const forbidden = toolDefinitions(forbiddenBroker, "boolean", "visualization_forbidden").find(
    (tool) => tool.name === "read_generated_image",
  );
  assert.ok(forbidden);
  const forbiddenPending = forbidden.execute("tool-1", { path: "image.png" }, undefined, undefined, undefined as never);
  forbiddenBroker.reply({
    version: 2,
    type: "tool_reply",
    id: "tool-1",
    ok: true,
    result: { mime: "image/png", data: "iVBORw0KGgo=" },
  });
  const forbiddenResult = await forbiddenPending;
  assert.equal(forbiddenResult.content.length, 1);
  assert.equal(forbiddenResult.content[0].type, "image");

  const errorBroker = createBroker(() => undefined, 1);
  const encouraged = toolDefinitions(errorBroker, "boolean", "visualization_encouraged").find(
    (tool) => tool.name === "read_generated_image",
  );
  assert.ok(encouraged);
  const errorPending = encouraged.execute("tool-1", { path: "image.png" }, undefined, undefined, undefined as never);
  errorBroker.reply({ version: 2, type: "tool_reply", id: "tool-1", ok: false, error: "failed" });
  await assert.rejects(errorPending, /failed/);

  const retryBroker = createBroker(() => undefined, 1);
  const retry = toolDefinitions(retryBroker, "boolean", "visualization_encouraged").find(
    (tool) => tool.name === "read_generated_image",
  );
  assert.ok(retry);
  const retryPending = retry.execute("tool-1", { path: "image.png" }, undefined, undefined, undefined as never);
  retryBroker.reply({ version: 2, type: "tool_reply", id: "tool-1", ok: false, error: "image_read_failed_retry_once" });
  const retryResult = await retryPending;
  assert.deepEqual(retryResult.content, [{
    type: "text",
    text: "Image read failed. You have one final image-read attempt. Call read_generated_image once with the relative path of an existing valid PNG under /work. Do not call sandbox_exec or submit_answer.",
  }]);
});

test("encouraged second sandbox result appends only the static exhaustion cue", async () => {
  const frames: Array<Record<string, unknown>> = [];
  const broker = createBroker((frame) => frames.push(frame as Record<string, unknown>), 1);
  const sandbox = toolDefinitions(broker, "boolean", "visualization_encouraged").find((tool) => tool.name === "sandbox_exec");
  assert.ok(sandbox);
  const run = async (id: string) => {
    const pending = sandbox.execute(id, { command: "hidden" }, undefined, undefined, undefined as never);
    broker.reply({ version: 2, type: "tool_reply", id, ok: true, result: { text: "normal result" } });
    return pending;
  };
  const first = await run("tool-1");
  const second = await run("tool-2");
  assert.deepEqual(first.content, [{ type: "text", text: "normal result" }]);
  assert.deepEqual(second.content, [
    { type: "text", text: "normal result" },
    { type: "text", text: "Pre-image sandbox budget is exhausted. Your next and only permitted tool is read_generated_image using the relative PNG path you created. Do not call sandbox_exec or submit_answer." },
  ]);
  assert.equal(frames.length, 2);
});

test("acknowledgement is static and other tools remain unchanged", async () => {
  const broker = createBroker(() => undefined, 1);
  const submit = toolDefinitions(broker, "boolean", "visualization_encouraged").find(
    (tool) => tool.name === "submit_answer",
  );
  assert.ok(submit);
  const pending = submit.execute("tool-1", { answer: true }, undefined, undefined, undefined as never);
  broker.reply({
    version: 2,
    type: "tool_reply",
    id: "tool-1",
    ok: true,
    result: { text: '{"accepted":true,"instance_id":"instance","answer_type":"boolean"}' },
  });
  const result = await pending;
  assert.deepEqual(result.content, [
    { type: "text", text: '{"accepted":true,"instance_id":"instance","answer_type":"boolean"}' },
  ]);
});
