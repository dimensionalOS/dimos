import {
  type EvalSocket,
  HttpMcpTransport,
  type McpTool,
  type McpTransport,
  runAgentEvalOnSocket,
} from "./agent-driver.ts";

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

class FakeSocket extends EventTarget implements EvalSocket {
  readyState: number = WebSocket.OPEN;
  sent: Array<Record<string, unknown>> = [];
  onSend?: (message: Record<string, unknown>) => void;

  send(data: string): void {
    const message = JSON.parse(data) as Record<string, unknown>;
    this.sent.push(message);
    this.onSend?.(message);
  }

  emit(message: Record<string, unknown>): void {
    this.dispatchEvent(
      new MessageEvent("message", { data: JSON.stringify(message) }),
    );
  }

  close(): void {
    this.readyState = WebSocket.CLOSED;
  }
}

class FakeMcp implements McpTransport {
  calls: Array<{ name: string; args?: Record<string, unknown> }> = [];

  constructor(
    readonly tools: McpTool[],
    readonly failTool?: string,
  ) {}

  listTools(_url: string, _timeoutMs: number): Promise<McpTool[]> {
    this.calls.push({ name: "tools/list" });
    return Promise.resolve(this.tools);
  }

  callTool(
    _url: string,
    name: string,
    args: Record<string, unknown>,
    _timeoutMs: number,
  ): Promise<unknown> {
    this.calls.push({ name, args });
    if (name === this.failTool) {
      return Promise.reject(new Error(`${name} failed`));
    }
    return Promise.resolve({ ok: true });
  }
}

const workflow = {
  scene: "apartment",
  workflow: "go-to-couch",
  url: "/scenes/apartment/evals/go-to-couch.js",
};

Deno.test("agent eval orders reset, one dispatch, start, result, and cleanup", async () => {
  const socket = new FakeSocket();
  const mcp = new FakeMcp([
    { name: "agent_send" },
    { name: "stop_navigation" },
  ]);
  const runId = "run-current";
  socket.onSend = (message) => {
    if (message.type === "runEval") {
      socket.emit({
        type: "evalReady",
        runId: "run-stale",
        workflowUrl: workflow.url,
        scene: workflow.scene,
        task: "wrong task",
        timeoutMs: 100,
        startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      });
      socket.emit({
        type: "evalReady",
        runId,
        workflowUrl: workflow.url,
        scene: workflow.scene,
        task: "Go to the couch",
        timeoutMs: 100,
        startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      });
    } else if (message.type === "evalReset") {
      socket.emit({
        type: "resetAck",
        runId,
        ok: true,
        pose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      });
    } else if (message.type === "evalStart") {
      socket.emit({
        type: "evalResult",
        runId,
        workflowUrl: workflow.url,
        scene: workflow.scene,
        task: "Go to the couch",
        passed: true,
        status: "passed",
        reason: "at couch",
        durationMs: 12,
      });
    }
  };

  const result = await runAgentEvalOnSocket({
    socket,
    workflow,
    mcpUrl: "http://127.0.0.1:9990/mcp",
    mcp,
    runId,
    timeouts: {
      browserReadyMs: 50,
      resetMs: 50,
      mcpMs: 50,
      resultGraceMs: 50,
    },
  });

  assert(result.passed, JSON.stringify(result));
  assertEquals(
    socket.sent.map((message) => message.type),
    ["runEval", "evalReset", "evalStart", "evalCleanup"],
  );
  assertEquals(
    mcp.calls,
    [
      { name: "tools/list" },
      { name: "agent_send", args: { message: "Go to the couch" } },
      { name: "stop_navigation", args: {} },
    ],
  );
  assertEquals(result.runId, runId);
});

Deno.test("agent eval browser-ready watchdog aborts and cleans up", async () => {
  const socket = new FakeSocket();
  const result = await runAgentEvalOnSocket({
    socket,
    workflow,
    mcpUrl: "http://127.0.0.1:9990/mcp",
    mcp: new FakeMcp([{ name: "agent_send" }]),
    runId: "watchdog-run",
    timeouts: { browserReadyMs: 5 },
  });

  assertEquals(result.status, "error");
  assertEquals(result.failureStage, "browserReady");
  assertEquals(
    socket.sent.map((message) => message.type),
    ["runEval", "evalAbort", "evalCleanup"],
  );
});

Deno.test("agent eval reset watchdog aborts before MCP dispatch", async () => {
  const socket = new FakeSocket();
  const mcp = new FakeMcp([{ name: "agent_send" }]);
  const runId = "reset-watchdog";
  socket.onSend = (message) => {
    if (message.type === "runEval") {
      socket.emit({
        type: "evalReady",
        runId,
        workflowUrl: workflow.url,
        scene: workflow.scene,
        task: "Go to the couch",
        timeoutMs: 100,
        startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      });
    }
  };

  const result = await runAgentEvalOnSocket({
    socket,
    workflow,
    mcpUrl: "http://127.0.0.1:9990/mcp",
    mcp,
    runId,
    timeouts: { browserReadyMs: 50, resetMs: 5 },
  });

  assertEquals(result.failureStage, "reset");
  assertEquals(mcp.calls, []);
  assertEquals(
    socket.sent.map((message) => message.type),
    ["runEval", "evalReset", "evalAbort", "evalCleanup"],
  );
});

Deno.test("agent eval MCP watchdog prevents evalStart", async () => {
  const socket = new FakeSocket();
  const runId = "mcp-watchdog";
  const mcp: McpTransport = {
    listTools: () => new Promise(() => {}),
    callTool: () => Promise.resolve(),
  };
  socket.onSend = (message) => {
    if (message.type === "runEval") {
      socket.emit({
        type: "evalReady",
        runId,
        workflowUrl: workflow.url,
        scene: workflow.scene,
        task: "Go to the couch",
        timeoutMs: 100,
        startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      });
    } else if (message.type === "evalReset") {
      socket.emit({
        type: "resetAck",
        runId,
        ok: true,
        pose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      });
    }
  };

  const result = await runAgentEvalOnSocket({
    socket,
    workflow,
    mcpUrl: "http://127.0.0.1:9990/mcp",
    mcp,
    runId,
    timeouts: { browserReadyMs: 50, resetMs: 50, mcpMs: 5 },
  });

  assertEquals(result.failureStage, "mcp");
  assert(!socket.sent.some((message) => message.type === "evalStart"));
});

Deno.test("agent eval result watchdog uses workflow timeout plus grace", async () => {
  const socket = new FakeSocket();
  const runId = "result-watchdog";
  socket.onSend = (message) => {
    if (message.type === "runEval") {
      socket.emit({
        type: "evalReady",
        runId,
        workflowUrl: workflow.url,
        scene: workflow.scene,
        task: "Go to the couch",
        timeoutMs: 5,
        startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      });
    } else if (message.type === "evalReset") {
      socket.emit({
        type: "resetAck",
        runId,
        ok: true,
        pose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      });
    }
  };

  const result = await runAgentEvalOnSocket({
    socket,
    workflow,
    mcpUrl: "http://127.0.0.1:9990/mcp",
    mcp: new FakeMcp([{ name: "agent_send" }]),
    runId,
    timeouts: {
      browserReadyMs: 50,
      resetMs: 50,
      mcpMs: 50,
      resultGraceMs: 5,
    },
  });

  assertEquals(result.failureStage, "result");
  assert(socket.sent.some((message) => message.type === "evalStart"));
});

Deno.test("agent eval MCP failure never starts scoring and dispatches once", async () => {
  const socket = new FakeSocket();
  const mcp = new FakeMcp([{ name: "agent_send" }], "agent_send");
  const runId = "mcp-failure";
  socket.onSend = (message) => {
    if (message.type === "runEval") {
      socket.emit({
        type: "evalReady",
        runId,
        workflowUrl: workflow.url,
        scene: workflow.scene,
        task: "Go to the couch",
        timeoutMs: 100,
        startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      });
    } else if (message.type === "evalReset") {
      socket.emit({
        type: "resetAck",
        runId,
        ok: true,
        pose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      });
    }
  };

  const result = await runAgentEvalOnSocket({
    socket,
    workflow,
    mcpUrl: "http://127.0.0.1:9990/mcp",
    mcp,
    runId,
    timeouts: { browserReadyMs: 50, resetMs: 50, mcpMs: 50 },
  });

  assertEquals(result.failureStage, "mcp");
  assertEquals(
    socket.sent.map((message) => message.type),
    ["runEval", "evalReset", "evalAbort", "evalCleanup"],
  );
  assertEquals(
    mcp.calls.filter((call) => call.name === "agent_send").length,
    1,
  );
});

Deno.test("HTTP MCP transport surfaces JSON-RPC errors", async () => {
  const originalFetch = globalThis.fetch;
  globalThis.fetch = (() =>
    Promise.resolve(
      new Response(
        JSON.stringify({
          jsonrpc: "2.0",
          id: 1,
          error: { code: -32603, message: "boom" },
        }),
        { status: 200, headers: { "content-type": "application/json" } },
      ),
    )) as typeof fetch;
  try {
    let caught: unknown;
    try {
      await new HttpMcpTransport().listTools("http://mcp.invalid/mcp", 50);
    } catch (error) {
      caught = error;
    }
    assert(caught instanceof Error);
    assert(caught.message.includes("boom"));
  } finally {
    globalThis.fetch = originalFetch;
  }
});

Deno.test("HTTP MCP transport surfaces DimOS tool error content", async () => {
  const originalFetch = globalThis.fetch;
  globalThis.fetch = (() =>
    Promise.resolve(
      new Response(
        JSON.stringify({
          jsonrpc: "2.0",
          id: 1,
          result: {
            content: [{
              type: "text",
              text: "Error running tool 'agent_send': transport unavailable",
            }],
          },
        }),
        { status: 200, headers: { "content-type": "application/json" } },
      ),
    )) as typeof fetch;
  try {
    let caught: unknown;
    try {
      await new HttpMcpTransport().callTool(
        "http://mcp.invalid/mcp",
        "agent_send",
        { message: "Go to the couch" },
        50,
      );
    } catch (error) {
      caught = error;
    }
    assert(caught instanceof Error);
    assert(caught.message.includes("transport unavailable"));
  } finally {
    globalThis.fetch = originalFetch;
  }
});
