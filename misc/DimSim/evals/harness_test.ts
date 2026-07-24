import { EvalHarness, type EvalWorkflow } from "./harness.ts";

function assert(
  condition: unknown,
  message = "assertion failed",
): asserts condition {
  if (!condition) throw new Error(message);
}

function assertEquals(actual: unknown, expected: unknown): void {
  const a = JSON.stringify(actual);
  const e = JSON.stringify(expected);
  if (a !== e) throw new Error(`expected ${e}, got ${a}`);
}

class FakeBridge {
  ws = null;
  sent: Array<Record<string, unknown>> = [];
  connect(): void {}
  sendCommand(message: Record<string, unknown>): void {
    this.sent.push({ ...message });
  }
}

function makeHarness(): { harness: EvalHarness; bridge: FakeBridge } {
  const bridge = new FakeBridge();
  const harness = new EvalHarness({
    bridge: bridge as any,
    getSceneState: () => ({ assets: [] }),
    getAgentPose: () => ({ x: 9, y: 0.5, z: 9, yaw: 0, pitch: 0 }),
  });
  (harness as any)._showOverlay = () => {};
  (harness as any)._showResult = () => {};
  return { harness, bridge };
}

function setAgentCommand(harness: EvalHarness, runId: string): void {
  (harness as any)._command = {
    runId,
    workflowUrl: "/scenes/apartment/evals/test.js",
    agent: true,
  };
}

const originalWindow = (globalThis as any).window;

Deno.test({
  name: "agent harness does not start timer before evalStart",
  async fn() {
    (globalThis as any).window = { __dimosAgent: null };
    try {
      const { harness, bridge } = makeHarness();
      setAgentCommand(harness, "timer-run");
      let rubricCalls = 0;
      const workflow: EvalWorkflow = {
        scene: "apartment",
        task: "Go somewhere",
        timeoutSec: 1,
        startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
        success: () => {
          rubricCalls++;
          return {
            passed: rubricCalls >= 2,
            reason: rubricCalls >= 2 ? "done" : "not yet",
          };
        },
      };

      const resultPromise = harness.runEval(workflow);
      await new Promise((resolve) => setTimeout(resolve, 10));
      assertEquals(rubricCalls, 1);
      assertEquals(bridge.sent.map((message) => message.type), ["evalReady"]);

      await harness._handleCommand({ type: "evalStart", runId: "stale-run" });
      assertEquals(rubricCalls, 1);
      await harness._handleCommand({ type: "evalStart", runId: "timer-run" });
      const result = await resultPromise;

      assertEquals(rubricCalls, 2);
      assert(result.passed);
      assertEquals(
        bridge.sent.map((message) => message.type),
        ["evalReady", "evalResult"],
      );
    } finally {
      (globalThis as any).window = originalWindow;
    }
  },
  sanitizeOps: false,
  sanitizeResources: false,
});

Deno.test("agent harness classifies setup failures as infrastructure errors", async () => {
  (globalThis as any).window = { __dimosAgent: null };
  try {
    const { harness, bridge } = makeHarness();
    setAgentCommand(harness, "setup-run");
    const result = await harness.runEval({
      scene: "apartment",
      task: "Go somewhere",
      startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      setup: () => {
        throw new Error("setup boom");
      },
      success: () => ({ passed: false }),
    });
    assertEquals(result.status, "error");
    assertEquals(result.failureStage, "setup");
    assertEquals(bridge.sent[0].status, "error");
  } finally {
    (globalThis as any).window = originalWindow;
  }
});

Deno.test("agent harness rejects an initially satisfied rubric", async () => {
  (globalThis as any).window = { __dimosAgent: null };
  try {
    const { harness, bridge } = makeHarness();
    setAgentCommand(harness, "initial-run");
    const result = await harness.runEval({
      scene: "apartment",
      task: "Already done",
      startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      success: ({ agentPos }) => ({
        passed: agentPos.x === 0 && agentPos.z === 3,
      }),
    });
    assertEquals(result.status, "error");
    assertEquals(result.failureStage, "initialRubric");
    assertEquals(bridge.sent.map((message) => message.type), ["evalResult"]);
  } finally {
    (globalThis as any).window = originalWindow;
  }
});

Deno.test("agent harness remembers aborts received during setup", async () => {
  (globalThis as any).window = { __dimosAgent: null };
  try {
    const { harness, bridge } = makeHarness();
    setAgentCommand(harness, "slow-setup-run");
    let releaseSetup: (() => void) | undefined;
    const setupGate = new Promise<void>((resolve) => {
      releaseSetup = resolve;
    });
    const resultPromise = harness.runEval({
      scene: "apartment",
      task: "Slow setup",
      startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      setup: () => setupGate,
      success: () => ({ passed: false }),
    });

    await new Promise((resolve) => setTimeout(resolve, 0));
    await harness._handleCommand({
      type: "evalAbort",
      runId: "slow-setup-run",
      reason: "browserReady timed out",
      failureStage: "browserReady",
    });
    releaseSetup!();
    const result = await resultPromise;

    assertEquals(result.status, "error");
    assertEquals(result.failureStage, "browserReady");
    assertEquals(bridge.sent, []);
    assert((harness as any)._pendingAgentEval === null);
  } finally {
    (globalThis as any).window = originalWindow;
  }
});

Deno.test("agent harness classifies workflow import failures as errors", async () => {
  const { harness, bridge } = makeHarness();
  await harness._loadAndRunWorkflowFile({
    type: "runEval",
    runId: "import-run",
    workflowUrl: "file:///definitely/missing/dimsim-workflow.js",
    agent: true,
  });
  assertEquals(bridge.sent[0].status, "error");
  assertEquals(bridge.sent[0].failureStage, "import");
});
