import {
  type EvalSocket,
  HttpMcpTransport,
  type McpTool,
  type McpTransport,
  runAgentEvalOnSocket,
} from "./agent-driver.ts";
import type {
  AgentIdleEvent,
  AgentOutputEvent,
  AgentOutputObserver,
} from "./agent-output.ts";

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
  autoPhysicsReady = true;

  send(data: string): void {
    const message = JSON.parse(data) as Record<string, unknown>;
    this.sent.push(message);
    this.onSend?.(message);
    if (message.type === "physicsReadyRequest" && this.autoPhysicsReady) {
      queueMicrotask(() => this.emit({ type: "physicsReady" }));
    }
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
    readonly onCall?: (
      name: string,
      args: Record<string, unknown>,
    ) => void,
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
    this.onCall?.(name, args);
    if (name === this.failTool) {
      return Promise.reject(new Error(`${name} failed`));
    }
    return Promise.resolve({ ok: true });
  }
}

class FakeAgentOutputObserver implements AgentOutputObserver {
  started = false;
  stopped = false;
  cancelledRunIds: string[] = [];
  private handler?: (event: AgentOutputEvent) => void;
  private idleHandler?: (event: AgentIdleEvent) => void;
  private failObserver!: (error: Error) => void;
  readonly failure: Promise<Error>;

  constructor(
    readonly startError?: Error,
    readonly onStart?: (observer: FakeAgentOutputObserver) => void,
    readonly cancelError?: Error,
    readonly onCancel?: (
      runId: string,
      observer: FakeAgentOutputObserver,
    ) => void,
  ) {
    this.failure = new Promise((resolve) => {
      this.failObserver = resolve;
    });
  }

  start(
    handler: (event: AgentOutputEvent) => void,
    idleHandler?: (event: AgentIdleEvent) => void,
  ): Promise<void> {
    this.started = true;
    this.handler = handler;
    this.idleHandler = idleHandler;
    if (this.startError) return Promise.reject(this.startError);
    this.onStart?.(this);
    return Promise.resolve();
  }

  emitIdle(idle: boolean, timestampMs = 1234): void {
    this.idleHandler?.({
      type: "agent_idle",
      idle,
      timestampMs,
    });
  }

  emit(
    text: string,
    hasToolCalls = false,
    timestampMs = 1234,
  ): void {
    this.handler?.({
      type: "agent_output",
      text,
      hasToolCalls,
      timestampMs,
    });
  }

  fail(message: string): void {
    this.failObserver(new Error(message));
  }

  cancelActiveTurn(runId: string): Promise<void> {
    this.cancelledRunIds.push(runId);
    this.onCancel?.(runId, this);
    if (this.cancelError) return Promise.reject(this.cancelError);
    return Promise.resolve();
  }

  stop(): Promise<void> {
    this.stopped = true;
    return Promise.resolve();
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
      sensorSettleMs: 0,
      agentIdleProbeMs: 0,
      mcpMs: 50,
      resultGraceMs: 50,
    },
  });

  assert(result.passed, JSON.stringify(result));
  assertEquals(
    socket.sent.map((message) => message.type),
    [
      "physicsReadyRequest",
      "runEval",
      "evalReset",
      "evalStart",
      "evalCleanup",
    ],
  );
  assertEquals(
    mcp.calls,
    [
      { name: "tools/list" },
      {
        name: "agent_send",
        args: { message: "Go to the couch" },
      },
      { name: "stop_navigation", args: {} },
    ],
  );
  assertEquals(result.runId, runId);
});

Deno.test("agent eval fences dispatch and terminal cleanup with DimSim idle state", async () => {
  const socket = new FakeSocket();
  const observer = new FakeAgentOutputObserver(
    undefined,
    (started) => started.emitIdle(true),
    undefined,
    (_runId, cancelling) => cancelling.emitIdle(true),
  );
  const mcp = new FakeMcp(
    [
      { name: "agent_send" },
      { name: "stop_navigation" },
    ],
    undefined,
    (name) => {
      if (name === "agent_send") observer.emitIdle(false);
    },
  );
  const runId = "idle-fenced-run";
  socket.onSend = (message) => {
    if (message.type === "runEval") {
      socket.emit({
        type: "evalReady",
        runId,
        workflowUrl: workflow.url,
        scene: workflow.scene,
        task: "Go to the couch",
        timeoutMs: 100,
        startPose: { x: 1, y: 0.5, z: 3, yaw: 117 },
      });
    } else if (message.type === "evalReset") {
      socket.emit({
        type: "resetAck",
        runId,
        ok: true,
        pose: { x: 1, y: 0.5, z: 3, yaw: 117 },
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
    agentOutputObserverFactory: () => observer,
    timeouts: {
      browserReadyMs: 50,
      resetMs: 50,
      sensorSettleMs: 0,
      agentOutputMs: 50,
      agentIdleProbeMs: 50,
      agentDispatchMs: 50,
      agentIdleMs: 50,
      mcpMs: 50,
      resultGraceMs: 50,
    },
  });

  assert(result.passed, JSON.stringify(result));
  assert(observer.started);
  assert(observer.stopped);
  assertEquals(observer.cancelledRunIds, [runId]);
  assertEquals(
    socket.sent.map((message) => message.type),
    [
      "physicsReadyRequest",
      "runEval",
      "evalReset",
      "evalStart",
      "evalCleanup",
    ],
  );
  assertEquals(
    mcp.calls.map((call) => call.name),
    ["tools/list", "agent_send", "stop_navigation"],
  );
});

Deno.test("agent eval reports an isolation error when turn cancellation cannot stop the active turn", async () => {
  const socket = new FakeSocket();
  const observer = new FakeAgentOutputObserver(
    undefined,
    (started) => started.emitIdle(true),
    new Error("control channel unavailable"),
  );
  const mcp = new FakeMcp(
    [{ name: "agent_send" }, { name: "stop_navigation" }],
    undefined,
    (name) => {
      if (name === "agent_send") observer.emitIdle(false);
    },
  );
  const runId = "turn-cancel-failure";
  socket.onSend = (message) => {
    if (message.type === "runEval") {
      socket.emit({
        type: "evalReady",
        runId,
        workflowUrl: workflow.url,
        scene: workflow.scene,
        task: "Go to the couch",
        timeoutMs: 100,
        startPose: { x: 1, y: 0.5, z: 3, yaw: 117 },
      });
    } else if (message.type === "evalReset") {
      socket.emit({
        type: "resetAck",
        runId,
        ok: true,
        pose: { x: 1, y: 0.5, z: 3, yaw: 117 },
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
    agentOutputObserverFactory: () => observer,
    timeouts: {
      browserReadyMs: 50,
      resetMs: 50,
      sensorSettleMs: 0,
      agentOutputMs: 50,
      agentIdleProbeMs: 50,
      agentDispatchMs: 20,
      agentIdleMs: 20,
      mcpMs: 50,
      resultGraceMs: 50,
    },
  });

  assertEquals(result.status, "error");
  assertEquals(result.failureStage, "agentIdle");
  assert(result.reason.includes("timed out"), result.reason);
  assertEquals(observer.cancelledRunIds, [runId]);
});

Deno.test("agent eval refuses to start scoring without a correlated busy edge", async () => {
  const socket = new FakeSocket();
  const observer = new FakeAgentOutputObserver(
    undefined,
    (started) => started.emitIdle(true),
  );
  const runId = "missing-busy-edge";
  socket.onSend = (message) => {
    if (message.type === "runEval") {
      socket.emit({
        type: "evalReady",
        runId,
        workflowUrl: workflow.url,
        scene: workflow.scene,
        task: "Go to the couch",
        timeoutMs: 100,
        startPose: { x: 1, y: 0.5, z: 3, yaw: 117 },
      });
    } else if (message.type === "evalReset") {
      socket.emit({
        type: "resetAck",
        runId,
        ok: true,
        pose: { x: 1, y: 0.5, z: 3, yaw: 117 },
      });
    }
  };

  const result = await runAgentEvalOnSocket({
    socket,
    workflow,
    mcpUrl: "http://127.0.0.1:9990/mcp",
    mcp: new FakeMcp([{ name: "agent_send" }]),
    runId,
    agentOutputObserverFactory: () => observer,
    timeouts: {
      browserReadyMs: 50,
      resetMs: 50,
      sensorSettleMs: 0,
      agentOutputMs: 50,
      agentIdleProbeMs: 50,
      agentDispatchMs: 5,
      agentIdleMs: 5,
      mcpMs: 50,
    },
  });

  assertEquals(result.failureStage, "agentIdle");
  assert(!socket.sent.some((message) => message.type === "evalStart"));
  assert(observer.stopped);
});

Deno.test("agent eval forwards only exact tool-free required output and owns sidecar", async () => {
  const socket = new FakeSocket();
  const observer = new FakeAgentOutputObserver(
    undefined,
    (started) => started.emit("FOUND_BATHTUB"),
  );
  const mcp = new FakeMcp([
    { name: "agent_send" },
    { name: "end_exploration" },
    { name: "stop_looking_out" },
    { name: "stop_navigation" },
  ]);
  const runId = "required-output-run";
  socket.onSend = (message) => {
    if (message.type === "runEval") {
      socket.emit({
        type: "evalReady",
        runId,
        workflowUrl: workflow.url,
        scene: workflow.scene,
        task: "Find the bathtub",
        timeoutMs: 100,
        startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
        requiredAgentOutput: "FOUND_BATHTUB",
      });
    } else if (message.type === "evalReset") {
      socket.emit({
        type: "resetAck",
        runId,
        ok: true,
        pose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      });
    } else if (message.type === "evalStart") {
      observer.emit("FOUND_BATHTUB", true);
      observer.emit("I found it: FOUND_BATHTUB");
      observer.emit("  FOUND_BATHTUB  ", false, 5678);
    } else if (message.type === "evalAgentOutput") {
      socket.emit({
        type: "evalResult",
        runId,
        workflowUrl: workflow.url,
        scene: workflow.scene,
        task: "Find the bathtub",
        passed: true,
        status: "passed",
        reason: "declared and nearby",
        durationMs: 12,
        evidence: {
          agentOutput: {
            text: "FOUND_BATHTUB",
            timestampMs: 5678,
            pose: { x: 1, y: 0.5, z: 2, yaw: 0 },
          },
        },
      });
    }
  };

  const result = await runAgentEvalOnSocket({
    socket,
    workflow,
    mcpUrl: "http://127.0.0.1:9990/mcp",
    mcp,
    runId,
    agentOutputObserverFactory: () => observer,
    timeouts: {
      browserReadyMs: 50,
      resetMs: 50,
      sensorSettleMs: 0,
      agentOutputMs: 50,
      agentIdleProbeMs: 0,
      mcpMs: 50,
      resultGraceMs: 50,
    },
  });

  assert(result.passed, JSON.stringify(result));
  assert(observer.started);
  assert(observer.stopped);
  assertEquals(
    socket.sent.map((message) => message.type),
    [
      "physicsReadyRequest",
      "runEval",
      "evalReset",
      "evalStart",
      "evalAgentOutput",
      "evalCleanup",
    ],
  );
  assertEquals(
    mcp.calls.map((call) => call.name),
    [
      "tools/list",
      "agent_send",
      "end_exploration",
      "stop_looking_out",
      "stop_navigation",
    ],
  );
  assertEquals(result.evidence?.agentOutput?.timestampMs, 5678);
});

Deno.test("agent eval classifies sidecar startup failure before dispatch", async () => {
  const socket = new FakeSocket();
  const observer = new FakeAgentOutputObserver(
    new Error("observer unavailable"),
  );
  const mcp = new FakeMcp([{ name: "agent_send" }]);
  const runId = "sidecar-start-failure";
  socket.onSend = (message) => {
    if (message.type === "runEval") {
      socket.emit({
        type: "evalReady",
        runId,
        workflowUrl: workflow.url,
        scene: workflow.scene,
        task: "Find the bathtub",
        timeoutMs: 100,
        startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
        requiredAgentOutput: "FOUND_BATHTUB",
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
    agentOutputObserverFactory: () => observer,
    timeouts: {
      browserReadyMs: 50,
      resetMs: 50,
      sensorSettleMs: 0,
      agentOutputMs: 50,
      agentIdleProbeMs: 0,
    },
  });

  assertEquals(result.failureStage, "agentOutput");
  assertEquals(mcp.calls, []);
  assert(observer.stopped);
  assert(!socket.sent.some((message) => message.type === "evalStart"));
});

Deno.test("agent eval aborts if required-output sidecar exits during scoring", async () => {
  const socket = new FakeSocket();
  const observer = new FakeAgentOutputObserver();
  const runId = "sidecar-runtime-failure";
  socket.onSend = (message) => {
    if (message.type === "runEval") {
      socket.emit({
        type: "evalReady",
        runId,
        workflowUrl: workflow.url,
        scene: workflow.scene,
        task: "Find the bathtub",
        timeoutMs: 100,
        startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
        requiredAgentOutput: "FOUND_BATHTUB",
      });
    } else if (message.type === "evalReset") {
      socket.emit({
        type: "resetAck",
        runId,
        ok: true,
        pose: { x: 0, y: 0.5, z: 3, yaw: 0 },
      });
    } else if (message.type === "evalStart") {
      observer.fail("observer exited");
    }
  };

  const result = await runAgentEvalOnSocket({
    socket,
    workflow,
    mcpUrl: "http://127.0.0.1:9990/mcp",
    mcp: new FakeMcp([{ name: "agent_send" }]),
    runId,
    agentOutputObserverFactory: () => observer,
    timeouts: {
      browserReadyMs: 50,
      resetMs: 50,
      sensorSettleMs: 0,
      agentOutputMs: 50,
      agentIdleProbeMs: 0,
      mcpMs: 50,
      resultGraceMs: 50,
    },
  });

  assertEquals(result.failureStage, "agentOutput");
  assert(observer.stopped);
  assert(
    socket.sent.some((message) =>
      message.type === "evalAbort" &&
      message.failureStage === "agentOutput"
    ),
  );
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
    ["physicsReadyRequest", "runEval", "evalAbort", "evalCleanup"],
  );
});

Deno.test("agent eval waits for delayed bridge physics before runEval", async () => {
  const socket = new FakeSocket();
  socket.autoPhysicsReady = false;
  socket.onSend = (message) => {
    if (message.type === "physicsReadyRequest") {
      setTimeout(() => socket.emit({ type: "physicsReady" }), 5);
    }
  };

  const result = await runAgentEvalOnSocket({
    socket,
    workflow,
    mcpUrl: "http://127.0.0.1:9990/mcp",
    mcp: new FakeMcp([{ name: "agent_send" }]),
    runId: "delayed-physics",
    timeouts: {
      physicsReadyMs: 50,
      browserReadyMs: 5,
    },
  });

  assertEquals(result.failureStage, "browserReady");
  assertEquals(
    socket.sent.map((message) => message.type),
    ["physicsReadyRequest", "runEval", "evalAbort", "evalCleanup"],
  );
});

Deno.test("agent eval physics-ready watchdog aborts before runEval", async () => {
  const socket = new FakeSocket();
  socket.autoPhysicsReady = false;

  const result = await runAgentEvalOnSocket({
    socket,
    workflow,
    mcpUrl: "http://127.0.0.1:9990/mcp",
    mcp: new FakeMcp([{ name: "agent_send" }]),
    runId: "physics-watchdog",
    timeouts: { physicsReadyMs: 5 },
  });

  assertEquals(result.status, "error");
  assertEquals(result.failureStage, "physicsReady");
  assertEquals(
    socket.sent.map((message) => message.type),
    ["physicsReadyRequest", "evalAbort", "evalCleanup"],
  );
});

Deno.test("agent eval reports socket failure while waiting for physics", async () => {
  const socket = new FakeSocket();
  socket.autoPhysicsReady = false;
  socket.onSend = (message) => {
    if (message.type === "physicsReadyRequest") {
      queueMicrotask(() => socket.dispatchEvent(new Event("error")));
    }
  };

  const result = await runAgentEvalOnSocket({
    socket,
    workflow,
    mcpUrl: "http://127.0.0.1:9990/mcp",
    mcp: new FakeMcp([{ name: "agent_send" }]),
    runId: "physics-socket-failure",
    timeouts: { physicsReadyMs: 50 },
  });

  assertEquals(result.status, "error");
  assertEquals(result.failureStage, "socket");
  assertEquals(
    socket.sent.map((message) => message.type),
    ["physicsReadyRequest", "evalAbort", "evalCleanup"],
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
    timeouts: {
      browserReadyMs: 50,
      resetMs: 5,
      agentIdleProbeMs: 0,
    },
  });

  assertEquals(result.failureStage, "reset");
  assertEquals(mcp.calls, []);
  assertEquals(
    socket.sent.map((message) => message.type),
    [
      "physicsReadyRequest",
      "runEval",
      "evalReset",
      "evalAbort",
      "evalCleanup",
    ],
  );
});

Deno.test("agent eval rejects a finite reset acknowledgement at the wrong pose", async () => {
  const socket = new FakeSocket();
  const mcp = new FakeMcp([{ name: "agent_send" }]);
  const runId = "wrong-reset-pose";
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
        pose: { x: 2, y: 0.5, z: 3, yaw: 0 },
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
      sensorSettleMs: 0,
      agentIdleProbeMs: 0,
      mcpMs: 50,
    },
  });

  assertEquals(result.failureStage, "reset");
  assertEquals(mcp.calls, []);
  assert(!socket.sent.some((message) => message.type === "evalStart"));
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
    timeouts: {
      browserReadyMs: 50,
      resetMs: 50,
      sensorSettleMs: 0,
      agentIdleProbeMs: 0,
      mcpMs: 5,
    },
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
      sensorSettleMs: 0,
      agentIdleProbeMs: 0,
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
    timeouts: {
      browserReadyMs: 50,
      resetMs: 50,
      sensorSettleMs: 0,
      agentIdleProbeMs: 0,
      mcpMs: 50,
    },
  });

  assertEquals(result.failureStage, "mcp");
  assertEquals(
    socket.sent.map((message) => message.type),
    [
      "physicsReadyRequest",
      "runEval",
      "evalReset",
      "evalAbort",
      "evalCleanup",
    ],
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
