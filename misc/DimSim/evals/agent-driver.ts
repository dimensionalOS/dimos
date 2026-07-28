import type {
  EvalAbortMessage,
  EvalAgentOutputMessage,
  EvalCleanupMessage,
  EvalEvidence,
  EvalFailureStage,
  EvalProtocolMessage,
  EvalReadyMessage,
  EvalResetMessage,
  EvalResultMessage,
  EvalStartMessage,
  ResetAckMessage,
  RunEvalMessage,
} from "./protocol.ts";
import { isFiniteStartPose } from "./protocol.ts";
import {
  type AgentOutputEvent,
  type AgentOutputObserver,
  createAgentOutputObserver,
} from "./agent-output.ts";

export interface EvalSocket extends EventTarget {
  readonly readyState: number;
  send(data: string): void;
  close(): void;
}

export interface McpTool {
  name: string;
}

export interface McpTransport {
  listTools(url: string, timeoutMs: number): Promise<McpTool[]>;
  callTool(
    url: string,
    name: string,
    args: Record<string, unknown>,
    timeoutMs: number,
  ): Promise<unknown>;
}

export interface AgentWorkflow {
  scene: string;
  workflow: string;
  url: string;
}

export interface AgentEvalTimeouts {
  browserReadyMs: number;
  resetMs: number;
  sensorSettleMs: number;
  agentOutputMs: number;
  mcpMs: number;
  resultGraceMs: number;
}

export const DEFAULT_AGENT_TIMEOUTS: AgentEvalTimeouts = {
  browserReadyMs: 30_000,
  resetMs: 5_000,
  sensorSettleMs: 10_000,
  agentOutputMs: 5_000,
  mcpMs: 10_000,
  resultGraceMs: 5_000,
};

const RESET_POSITION_TOLERANCE_M = 0.25;
const RESET_YAW_TOLERANCE_DEG = 5;

export interface AgentEvalResult {
  runId: string;
  scene: string;
  workflow: string;
  workflowUrl: string;
  task: string;
  passed: boolean;
  status: "passed" | "failed" | "error";
  failureStage?: EvalFailureStage;
  reason: string;
  score: number | null;
  durationMs: number;
  evidence?: EvalEvidence;
}

interface WaitOptions {
  runId: string;
  timeoutMs: number;
  stage: EvalFailureStage;
  types: Set<EvalProtocolMessage["type"]>;
  signal?: AbortSignal;
}

class AgentEvalError extends Error {
  constructor(
    message: string,
    readonly stage: EvalFailureStage,
  ) {
    super(message);
  }
}

/** Minimal JSON-over-HTTP client for the DimOS MCP endpoint. */
export class HttpMcpTransport implements McpTransport {
  private requestId = 0;

  async listTools(url: string, timeoutMs: number): Promise<McpTool[]> {
    const result = await this._request(url, "tools/list", {}, timeoutMs);
    if (!result || typeof result !== "object") {
      throw new Error("MCP tools/list returned an invalid result");
    }
    const tools = (result as { tools?: unknown }).tools;
    if (!Array.isArray(tools)) {
      throw new Error("MCP tools/list response is missing tools");
    }
    return tools.filter(
      (tool): tool is McpTool =>
        !!tool && typeof tool === "object" &&
        typeof (tool as { name?: unknown }).name === "string",
    );
  }

  async callTool(
    url: string,
    name: string,
    args: Record<string, unknown>,
    timeoutMs: number,
  ): Promise<unknown> {
    const result = await this._request(
      url,
      "tools/call",
      { name, arguments: args },
      timeoutMs,
    );
    if (result && typeof result === "object") {
      const toolResult = result as {
        isError?: unknown;
        content?: Array<{ type?: unknown; text?: unknown }>;
      };
      const text = toolResult.content
        ?.filter((item) =>
          item?.type === "text" && typeof item.text === "string"
        )
        .map((item) => item.text as string)
        .join("\n") ?? "";
      if (
        toolResult.isError === true ||
        /^(Error running tool|Tool not found:|Cannot start\b)/.test(text)
      ) {
        throw new Error(text || `MCP tool ${name} failed`);
      }
    }
    return result;
  }

  private async _request(
    url: string,
    method: string,
    params: Record<string, unknown>,
    timeoutMs: number,
  ): Promise<unknown> {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeoutMs);
    try {
      const response = await fetch(url, {
        method: "POST",
        headers: {
          "content-type": "application/json",
          "accept": "application/json, text/event-stream",
        },
        body: JSON.stringify({
          jsonrpc: "2.0",
          id: ++this.requestId,
          method,
          params,
        }),
        signal: controller.signal,
      });
      if (!response.ok) {
        throw new Error(`MCP ${method} failed with HTTP ${response.status}`);
      }
      const body = await response.json();
      if (body?.error) {
        const detail = body.error.message ?? JSON.stringify(body.error);
        throw new Error(`MCP ${method} error: ${detail}`);
      }
      if (!Object.hasOwn(body ?? {}, "result")) {
        throw new Error(`MCP ${method} response is missing result`);
      }
      return body.result;
    } catch (error) {
      if (controller.signal.aborted) {
        throw new Error(`MCP ${method} timed out after ${timeoutMs}ms`);
      }
      throw error;
    } finally {
      clearTimeout(timer);
    }
  }
}

function send(socket: EvalSocket, message: EvalProtocolMessage): void {
  socket.send(JSON.stringify(message));
}

function yawDeltaDeg(a: number, b: number): number {
  const delta = ((a - b + 180) % 360 + 360) % 360 - 180;
  return Math.abs(delta);
}

async function withTimeout<T>(
  operation: Promise<T>,
  timeoutMs: number,
  label: string,
): Promise<T> {
  let timer: ReturnType<typeof setTimeout> | undefined;
  try {
    return await Promise.race([
      operation,
      new Promise<T>((_resolve, reject) => {
        timer = setTimeout(
          () => reject(new Error(`${label} timed out after ${timeoutMs}ms`)),
          timeoutMs,
        );
      }),
    ]);
  } finally {
    if (timer !== undefined) clearTimeout(timer);
  }
}

function waitForMessage(
  socket: EvalSocket,
  options: WaitOptions,
): Promise<EvalProtocolMessage> {
  return new Promise((resolve, reject) => {
    let settled = false;
    const cleanup = () => {
      clearTimeout(timer);
      socket.removeEventListener("message", onMessage);
      socket.removeEventListener("error", onSocketFailure);
      socket.removeEventListener("close", onSocketFailure);
      options.signal?.removeEventListener("abort", onAbort);
    };
    const settle = (fn: () => void) => {
      if (settled) return;
      settled = true;
      cleanup();
      fn();
    };
    const onMessage = (event: Event) => {
      const data = (event as MessageEvent).data;
      if (typeof data !== "string") return;
      let message: EvalProtocolMessage;
      try {
        message = JSON.parse(data) as EvalProtocolMessage;
      } catch {
        return;
      }
      if (!message || message.runId !== options.runId) return;
      if (message.type === "evalResult") {
        settle(() => resolve(message));
        return;
      }
      if (!options.types.has(message.type)) return;
      settle(() => resolve(message));
    };
    const onSocketFailure = (event: Event) => {
      settle(() =>
        reject(
          new AgentEvalError(
            event.type === "close"
              ? "websocket closed during agent eval"
              : "websocket error during agent eval",
            "socket",
          ),
        )
      );
    };
    const onAbort = () => {
      settle(() =>
        reject(
          new AgentEvalError(
            `${options.stage} wait cancelled`,
            options.stage,
          ),
        )
      );
    };
    const timer = setTimeout(() => {
      settle(() =>
        reject(
          new AgentEvalError(
            `${options.stage} timed out after ${options.timeoutMs}ms`,
            options.stage,
          ),
        )
      );
    }, options.timeoutMs);
    socket.addEventListener("message", onMessage);
    socket.addEventListener("error", onSocketFailure);
    socket.addEventListener("close", onSocketFailure);
    options.signal?.addEventListener("abort", onAbort, { once: true });
  });
}

function normalizeResult(
  workflow: AgentWorkflow,
  runId: string,
  message: EvalResultMessage,
): AgentEvalResult {
  const status = message.status ??
    (message.passed ? "passed" : "failed");
  return {
    runId,
    scene: workflow.scene,
    workflow: workflow.workflow,
    workflowUrl: workflow.url,
    task: message.task ?? "",
    passed: status === "passed" && !!message.passed,
    status,
    failureStage: message.failureStage,
    reason: message.reason ?? (message.passed ? "ok" : "fail"),
    score: typeof message.score === "number" ? message.score : null,
    durationMs: typeof message.durationMs === "number" ? message.durationMs : 0,
    evidence: message.evidence,
  };
}

function errorResult(
  workflow: AgentWorkflow,
  runId: string,
  error: unknown,
): AgentEvalResult {
  const stage = error instanceof AgentEvalError ? error.stage : "mcp";
  return {
    runId,
    scene: workflow.scene,
    workflow: workflow.workflow,
    workflowUrl: workflow.url,
    task: "",
    passed: false,
    status: "error",
    failureStage: stage,
    reason: error instanceof Error ? error.message : String(error),
    score: null,
    durationMs: 0,
  };
}

/**
 * Run the correlated agent lifecycle on an already-open bridge socket.
 * The workflow task is learned from the browser and sent to `agent_send`
 * exactly once, after the authoritative bridge reset succeeds.
 */
export async function runAgentEvalOnSocket(options: {
  socket: EvalSocket;
  workflow: AgentWorkflow;
  mcpUrl: string;
  mcp?: McpTransport;
  agentOutputObserverFactory?: () => AgentOutputObserver;
  runId?: string;
  timeouts?: Partial<AgentEvalTimeouts>;
}): Promise<AgentEvalResult> {
  const {
    socket,
    workflow,
    mcpUrl,
    mcp = new HttpMcpTransport(),
    runId = crypto.randomUUID(),
  } = options;
  const timeouts = { ...DEFAULT_AGENT_TIMEOUTS, ...options.timeouts };
  let tools: McpTool[] = [];
  let terminalError: AgentEvalError | null = null;
  let outputObserver: AgentOutputObserver | null = null;
  let requiredAgentOutput: string | undefined;
  let captureAgentOutput = false;
  let scoringStarted = false;
  let outputForwarded = false;
  let pendingAgentOutput: AgentOutputEvent | null = null;
  let resultWaitAbort: AbortController | null = null;

  const forwardAgentOutput = (event: AgentOutputEvent) => {
    if (outputForwarded) return;
    const message: EvalAgentOutputMessage = {
      type: "evalAgentOutput",
      runId,
      text: event.text.trim(),
      timestampMs: event.timestampMs,
    };
    send(socket, message);
    outputForwarded = true;
  };

  const onAgentOutput = (event: AgentOutputEvent) => {
    if (
      !captureAgentOutput ||
      !requiredAgentOutput ||
      event.hasToolCalls ||
      event.text.trim() !== requiredAgentOutput ||
      pendingAgentOutput
    ) {
      return;
    }
    pendingAgentOutput = event;
    if (scoringStarted) forwardAgentOutput(event);
  };

  const raceObserverFailure = async <T>(operation: Promise<T>): Promise<T> => {
    if (!outputObserver) return await operation;
    return await Promise.race([
      operation,
      outputObserver.failure.then((error) => {
        throw new AgentEvalError(error.message, "agentOutput");
      }),
    ]);
  };

  try {
    const runMessage: RunEvalMessage = {
      type: "runEval",
      runId,
      workflowUrl: workflow.url,
      agent: true,
    };
    const readyPromise = waitForMessage(socket, {
      runId,
      timeoutMs: timeouts.browserReadyMs,
      stage: "browserReady",
      types: new Set(["evalReady"]),
    });
    send(socket, runMessage);
    const readyOrResult = await readyPromise;
    if (readyOrResult.type === "evalResult") {
      return normalizeResult(workflow, runId, readyOrResult);
    }
    const ready = readyOrResult as EvalReadyMessage;
    if (
      ready.workflowUrl !== workflow.url ||
      typeof ready.task !== "string" ||
      ready.task.length === 0 ||
      !Number.isFinite(ready.timeoutMs) ||
      ready.timeoutMs <= 0 ||
      !isFiniteStartPose(ready.startPose) ||
      (ready.requiredAgentOutput !== undefined &&
        (typeof ready.requiredAgentOutput !== "string" ||
          ready.requiredAgentOutput.trim().length === 0))
    ) {
      throw new AgentEvalError(
        "browser returned invalid evalReady data",
        "browserReady",
      );
    }
    requiredAgentOutput = ready.requiredAgentOutput?.trim();

    const reset: EvalResetMessage = {
      type: "evalReset",
      runId,
      startPose: ready.startPose,
    };
    const resetPromise = waitForMessage(socket, {
      runId,
      timeoutMs: timeouts.resetMs,
      stage: "reset",
      types: new Set(["resetAck"]),
    });
    send(socket, reset);
    const resetOrResult = await resetPromise;
    if (resetOrResult.type === "evalResult") {
      return normalizeResult(workflow, runId, resetOrResult);
    }
    const ack = resetOrResult as ResetAckMessage;
    if (!ack.ok || !isFiniteStartPose(ack.pose)) {
      throw new AgentEvalError(
        ack.reason || "bridge reset failed or returned an invalid pose",
        "reset",
      );
    }
    const resetPositionError = Math.hypot(
      ack.pose.x - ready.startPose.x,
      ack.pose.y - ready.startPose.y,
      ack.pose.z - ready.startPose.z,
    );
    const resetYawError = yawDeltaDeg(ack.pose.yaw, ready.startPose.yaw);
    if (
      resetPositionError > RESET_POSITION_TOLERANCE_M ||
      resetYawError > RESET_YAW_TOLERANCE_DEG
    ) {
      throw new AgentEvalError(
        `bridge reset acknowledgement mismatched requested pose: ` +
          `position error ${resetPositionError.toFixed(3)}m, ` +
          `yaw error ${resetYawError.toFixed(1)}deg`,
        "reset",
      );
    }

    // Camera frames are produced asynchronously by the browser and then
    // consumed by the Python agent. Give that pipeline a bounded interval to
    // replace any frame captured immediately before the authoritative reset.
    // Scoring still begins only after dispatch and evalStart below.
    if (timeouts.sensorSettleMs > 0) {
      console.error(
        `[agent-eval] waiting ${timeouts.sensorSettleMs}ms for a post-reset camera frame`,
      );
      await new Promise((resolve) =>
        setTimeout(resolve, timeouts.sensorSettleMs)
      );
      if (socket.readyState !== WebSocket.OPEN) {
        throw new AgentEvalError(
          "bridge socket closed while waiting for post-reset sensors",
          "socket",
        );
      }
    }

    if (requiredAgentOutput) {
      outputObserver = options.agentOutputObserverFactory?.() ??
        createAgentOutputObserver();
      try {
        await withTimeout(
          outputObserver.start(onAgentOutput),
          timeouts.agentOutputMs,
          "agent output sidecar",
        );
      } catch (error) {
        throw new AgentEvalError(
          error instanceof Error ? error.message : String(error),
          "agentOutput",
        );
      }
    }

    try {
      const mcpDeadline = Date.now() + timeouts.mcpMs;
      tools = await withTimeout(
        raceObserverFailure(mcp.listTools(mcpUrl, timeouts.mcpMs)),
        timeouts.mcpMs,
        "MCP tools/list",
      );
      if (!tools.some((tool) => tool.name === "agent_send")) {
        throw new Error(
          "MCP server does not advertise required tool agent_send",
        );
      }
      const remainingMs = mcpDeadline - Date.now();
      if (remainingMs <= 0) {
        throw new Error(`MCP stage timed out after ${timeouts.mcpMs}ms`);
      }
      captureAgentOutput = true;
      await withTimeout(
        raceObserverFailure(
          mcp.callTool(
            mcpUrl,
            "agent_send",
            { message: ready.task },
            remainingMs,
          ),
        ),
        remainingMs,
        "MCP agent_send",
      );
    } catch (error) {
      captureAgentOutput = false;
      if (error instanceof AgentEvalError) throw error;
      throw new AgentEvalError(
        error instanceof Error ? error.message : String(error),
        "mcp",
      );
    }

    const start: EvalStartMessage = { type: "evalStart", runId };
    resultWaitAbort = new AbortController();
    const resultPromise = waitForMessage(socket, {
      runId,
      timeoutMs: ready.timeoutMs + timeouts.resultGraceMs,
      stage: "result",
      types: new Set(["evalResult"]),
      signal: resultWaitAbort.signal,
    });
    send(socket, start);
    scoringStarted = true;
    if (pendingAgentOutput) forwardAgentOutput(pendingAgentOutput);
    const result = await raceObserverFailure(
      resultPromise as Promise<EvalResultMessage>,
    );
    captureAgentOutput = false;
    return normalizeResult(workflow, runId, result);
  } catch (error) {
    terminalError = error instanceof AgentEvalError
      ? error
      : new AgentEvalError(
        error instanceof Error ? error.message : String(error),
        "mcp",
      );
    return errorResult(workflow, runId, terminalError);
  } finally {
    captureAgentOutput = false;
    resultWaitAbort?.abort();
    if (terminalError) {
      try {
        const abort: EvalAbortMessage = {
          type: "evalAbort",
          runId,
          reason: terminalError.message,
          failureStage: terminalError.stage,
        };
        send(socket, abort);
      } catch {
        // Socket failures are already represented by the terminal result.
      }
    }
    for (
      const cleanupTool of [
        "end_exploration",
        "stop_looking_out",
        "stop_navigation",
      ]
    ) {
      if (!tools.some((tool) => tool.name === cleanupTool)) continue;
      try {
        await withTimeout(
          mcp.callTool(
            mcpUrl,
            cleanupTool,
            {},
            timeouts.mcpMs,
          ),
          timeouts.mcpMs,
          `MCP ${cleanupTool}`,
        );
      } catch (error) {
        console.error(
          `[runner] cleanup ${cleanupTool} failed: ${
            error instanceof Error ? error.message : String(error)
          }`,
        );
      }
    }
    try {
      const cleanup: EvalCleanupMessage = { type: "evalCleanup", runId };
      send(socket, cleanup);
    } catch {
      // Best effort: the bridge also clears motion when the eval socket closes.
    }
    if (outputObserver) {
      try {
        await outputObserver.stop();
      } catch (error) {
        console.error(
          `[runner] cleanup agent output sidecar failed: ${
            error instanceof Error ? error.message : String(error)
          }`,
        );
      }
    }
  }
}
