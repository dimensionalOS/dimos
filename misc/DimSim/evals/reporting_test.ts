import {
  connectEvalSocket,
  type EvalResult,
  exitCodeForResults,
  formatResults,
  toJunitXml,
} from "./runner.ts";

function assert(
  condition: unknown,
  message = "assertion failed",
): asserts condition {
  if (!condition) throw new Error(message);
}

function result(
  status: "passed" | "failed" | "error",
  reason: string,
): EvalResult {
  return {
    runId: `run-${status}`,
    scene: "apartment",
    workflow: status,
    workflowUrl: `/workflows/${status}.js`,
    task: "task",
    passed: status === "passed",
    status,
    failureStage: status === "error" ? "mcp" : undefined,
    reason,
    score: null,
    durationMs: 10,
  };
}

Deno.test("JSON reporting contains only the result document", () => {
  const output = formatResults([result("passed", "ok")], "json");
  const parsed = JSON.parse(output);
  assert(Array.isArray(parsed));
  assert(parsed[0].status === "passed");
  assert(parsed[0].runId === "run-passed");
});

Deno.test("JUnit distinguishes task failures from infrastructure errors", () => {
  const xml = toJunitXml([
    result("passed", "ok"),
    result("failed", "too far"),
    result("error", "MCP unavailable"),
  ]);
  assert(xml.includes('failures="1" errors="1"'));
  assert(xml.includes('<failure message="too far"/>'));
  assert(xml.includes('<error message="MCP unavailable"/>'));
});

Deno.test("exit codes classify pass, failure, and infrastructure error", () => {
  assert(exitCodeForResults([result("passed", "ok")]) === 0);
  assert(exitCodeForResults([result("failed", "no")]) === 1);
  assert(
    exitCodeForResults([
      result("failed", "no"),
      result("error", "infra"),
    ]) === 2,
  );
});

Deno.test("bridge connection watchdog closes an unresponsive socket", async () => {
  class DeadSocket extends EventTarget {
    readyState = WebSocket.CONNECTING;
    closed = false;
    send(_data: string): void {}
    close(): void {
      this.closed = true;
    }
  }
  const socket = new DeadSocket();
  let caught: unknown;
  try {
    await connectEvalSocket(
      "ws://bridge.invalid",
      5,
      () => socket,
    );
  } catch (error) {
    caught = error;
  }
  assert(caught instanceof Error);
  assert(caught.message.includes("timed out connecting"));
  assert(socket.closed);
});
