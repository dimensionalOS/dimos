/**
 * Deno-side eval orchestration.
 *
 * Scorer mode remains a simple runEval/evalResult exchange. Agent mode uses
 * the correlated lifecycle implemented in agent-driver.ts and dispatches the
 * browser-provided task only after the bridge acknowledges an authoritative
 * physics reset.
 */

import { resolve } from "@std/path";
import {
  type AgentEvalTimeouts,
  type EvalSocket,
  type McpTransport,
  runAgentEvalOnSocket,
} from "./agent-driver.ts";
import type {
  EvalFailureStage,
  EvalResultMessage,
  EvalStatus,
  RunEvalMessage,
} from "./protocol.ts";

export interface EvalResult {
  runId: string;
  scene: string;
  workflow: string;
  workflowUrl: string;
  task: string;
  passed: boolean;
  status: EvalStatus;
  failureStage?: EvalFailureStage;
  reason: string;
  score: number | null;
  durationMs: number;
}

export interface WorkflowEntry {
  scene: string;
  workflow: string;
  filePath: string;
  /** URL the browser uses to dynamic-import the workflow module. */
  url: string;
}

export type EvalSocketFactory = (url: string) => EvalSocket;

export interface RunEvalOptions {
  /** Control WebSocket URL (no `?ch=...`). */
  wsUrl: string;
  /** Absolute path to the scenes/ root. */
  scenesRoot: string;
  filterScene?: string;
  filterWorkflow?: string;
  agent?: boolean;
  mcpUrl?: string;
  mcp?: McpTransport;
  agentTimeouts?: Partial<AgentEvalTimeouts>;
  connectionTimeoutMs?: number;
  socketFactory?: EvalSocketFactory;
}

/** Walk `scenes/<env>/evals/*.js` and return one entry per workflow file. */
export function collectWorkflows(opts: {
  scenesRoot: string;
  filterScene?: string;
  filterWorkflow?: string;
}): WorkflowEntry[] {
  const { scenesRoot, filterScene, filterWorkflow } = opts;

  const out: WorkflowEntry[] = [];
  let sceneDirs: Deno.DirEntry[];
  try {
    sceneDirs = [...Deno.readDirSync(scenesRoot)];
  } catch {
    return out;
  }

  for (const sceneEnt of sceneDirs) {
    if (!sceneEnt.isDirectory) continue;
    const scene = sceneEnt.name;
    if (filterScene && filterScene !== scene) continue;

    const evalsDir = resolve(scenesRoot, scene, "evals");
    let workflowEnts: Deno.DirEntry[];
    try {
      workflowEnts = [...Deno.readDirSync(evalsDir)];
    } catch {
      continue;
    }

    for (const ent of workflowEnts) {
      if (!ent.isFile || !ent.name.endsWith(".js")) continue;
      const workflow = ent.name.slice(0, -3);
      if (filterWorkflow && filterWorkflow !== workflow) continue;

      out.push({
        scene,
        workflow,
        filePath: resolve(evalsDir, ent.name),
        url: `/scenes/${scene}/evals/${ent.name}`,
      });
    }
  }
  return out;
}

/** Run each workflow sequentially over one control WebSocket. */
export async function runEvals(options: RunEvalOptions): Promise<EvalResult[]> {
  const workflows = collectWorkflows(options);
  if (workflows.length === 0) {
    console.error("[runner] no workflows match filter");
    return [];
  }
  if (options.agent && workflows.length !== 1) {
    return [
      configurationResult(
        options.filterScene ?? "",
        options.filterWorkflow ?? "",
        `agent mode requires exactly one workflow; matched ${workflows.length}`,
      ),
    ];
  }

  console.error(`[runner] running ${workflows.length} workflow(s)…`);
  let socket: EvalSocket;
  try {
    socket = await connectEvalSocket(
      options.wsUrl,
      options.connectionTimeoutMs ?? 5_000,
      options.socketFactory,
    );
  } catch (error) {
    return workflows.map((workflow) =>
      infrastructureResult(
        workflow,
        crypto.randomUUID(),
        "connection",
        error instanceof Error ? error.message : String(error),
      )
    );
  }

  try {
    const results: EvalResult[] = [];
    for (const workflow of workflows) {
      console.error(`[runner] → ${workflow.scene}/${workflow.workflow}`);
      const result = options.agent
        ? await runAgentEvalOnSocket({
          socket,
          workflow,
          mcpUrl: options.mcpUrl ??
            "http://127.0.0.1:9990/mcp",
          mcp: options.mcp,
          timeouts: options.agentTimeouts,
        })
        : await runScorerEvalOnSocket(socket, workflow);
      results.push(result);
      const tag = result.status === "error"
        ? "ERROR"
        : result.passed
        ? "PASS"
        : "FAIL";
      console.error(
        `[runner]   ${tag} (${result.durationMs}ms): ${result.reason}`,
      );
    }
    return results;
  } finally {
    try {
      socket.close();
    } catch {
      // ignore close races
    }
  }
}

/** Parallel scorer-only variant — one control WS per browser page. */
export interface RunEvalsMultiPageOptions extends RunEvalOptions {
  channels: string[];
}

export async function runEvalsMultiPage(
  options: RunEvalsMultiPageOptions,
): Promise<EvalResult[]> {
  const workflows = collectWorkflows(options);
  if (workflows.length === 0 || options.channels.length === 0) return [];

  let sockets: EvalSocket[];
  try {
    sockets = await Promise.all(
      options.channels.map((channel) =>
        connectEvalSocket(
          `${options.wsUrl}/?channel=${encodeURIComponent(channel)}&ch=control`,
          options.connectionTimeoutMs ?? 5_000,
          options.socketFactory,
        )
      ),
    );
  } catch (error) {
    return workflows.map((workflow) =>
      infrastructureResult(
        workflow,
        crypto.randomUUID(),
        "connection",
        error instanceof Error ? error.message : String(error),
      )
    );
  }

  const queues: WorkflowEntry[][] = sockets.map(() => []);
  workflows.forEach((workflow, index) =>
    queues[index % queues.length].push(workflow)
  );

  try {
    const all = await Promise.all(
      sockets.map(async (socket, index) => {
        const out: EvalResult[] = [];
        for (const workflow of queues[index]) {
          console.error(
            `[runner:${options.channels[index]}] → ${workflow.scene}/${workflow.workflow}`,
          );
          out.push(await runScorerEvalOnSocket(socket, workflow));
        }
        return out;
      }),
    );
    return all.flat();
  } finally {
    for (const socket of sockets) {
      try {
        socket.close();
      } catch {
        // ignore close races
      }
    }
  }
}

/** JUnit emitter: task failures are failures; infrastructure issues are errors. */
export function toJunitXml(results: EvalResult[]): string {
  const lines: string[] = [];
  lines.push('<?xml version="1.0" encoding="UTF-8"?>');
  const failures = results.filter((result) => result.status === "failed").length;
  const errors = results.filter((result) => result.status === "error").length;
  lines.push(
    `<testsuite name="dimsim-evals" tests="${results.length}" ` +
    `failures="${failures}" errors="${errors}">`,
  );
  for (const result of results) {
    const name = `${result.scene}/${result.workflow}`;
    const time = (result.durationMs / 1000).toFixed(3);
    if (result.status === "passed") {
      lines.push(`  <testcase name="${_escape(name)}" time="${time}"/>`);
      continue;
    }
    const element = result.status === "error" ? "error" : "failure";
    lines.push(`  <testcase name="${_escape(name)}" time="${time}">`);
    lines.push(
      `    <${element} message="${_escape(result.reason)}"/>`,
    );
    lines.push("  </testcase>");
  }
  lines.push("</testsuite>");
  return lines.join("\n");
}

export function formatResults(
  results: EvalResult[],
  format: "json" | "junit",
): string {
  return format === "junit"
    ? toJunitXml(results)
    : JSON.stringify(results, null, 2);
}

export function exitCodeForResults(results: EvalResult[]): 0 | 1 | 2 {
  if (
    results.length === 0 ||
    results.some((result) => result.status === "error")
  ) {
    return 2;
  }
  return results.some((result) => result.status === "failed") ? 1 : 0;
}

export function configurationResult(
  scene: string,
  workflow: string,
  reason: string,
): EvalResult {
  return {
    runId: crypto.randomUUID(),
    scene,
    workflow,
    workflowUrl: "",
    task: "",
    passed: false,
    status: "error",
    failureStage: "configuration",
    reason,
    score: null,
    durationMs: 0,
  };
}

function infrastructureResult(
  workflow: WorkflowEntry,
  runId: string,
  failureStage: EvalFailureStage,
  reason: string,
): EvalResult {
  return {
    runId,
    scene: workflow.scene,
    workflow: workflow.workflow,
    workflowUrl: workflow.url,
    task: "",
    passed: false,
    status: "error",
    failureStage,
    reason,
    score: null,
    durationMs: 0,
  };
}

export function connectEvalSocket(
  wsUrl: string,
  timeoutMs = 5_000,
  socketFactory: EvalSocketFactory = (url) => new WebSocket(url),
): Promise<EvalSocket> {
  const url = wsUrl.includes("?") ? wsUrl : `${wsUrl}/?ch=control`;
  return new Promise((resolve, reject) => {
    const socket = socketFactory(url);
    let settled = false;
    const cleanup = () => {
      clearTimeout(timer);
      socket.removeEventListener("open", onOpen);
      socket.removeEventListener("error", onError);
      socket.removeEventListener("close", onClose);
    };
    const onOpen = () => {
      if (settled) return;
      settled = true;
      cleanup();
      resolve(socket);
    };
    const onError = () => {
      if (settled) return;
      settled = true;
      cleanup();
      reject(new Error(`websocket error connecting to ${url}`));
    };
    const onClose = () => {
      if (settled) return;
      settled = true;
      cleanup();
      reject(new Error(`websocket closed while connecting to ${url}`));
    };
    const timer = setTimeout(() => {
      if (settled) return;
      settled = true;
      cleanup();
      try {
        socket.close();
      } catch {
        // ignore
      }
      reject(new Error(`timed out connecting to ${url} after ${timeoutMs}ms`));
    }, timeoutMs);
    socket.addEventListener("open", onOpen);
    socket.addEventListener("error", onError);
    socket.addEventListener("close", onClose);
  });
}

export function runScorerEvalOnSocket(
  socket: EvalSocket,
  workflow: WorkflowEntry,
  timeoutMs = 125_000,
  runId = crypto.randomUUID(),
): Promise<EvalResult> {
  return new Promise((resolve) => {
    let settled = false;
    const cleanup = () => {
      clearTimeout(timer);
      socket.removeEventListener("message", onMessage);
      socket.removeEventListener("error", onFailure);
      socket.removeEventListener("close", onFailure);
    };
    const finish = (result: EvalResult) => {
      if (settled) return;
      settled = true;
      cleanup();
      resolve(result);
    };
    const onMessage = (event: Event) => {
      const data = (event as MessageEvent).data;
      if (typeof data !== "string") return;
      let message: EvalResultMessage;
      try {
        message = JSON.parse(data) as EvalResultMessage;
      } catch {
        return;
      }
      if (message.type !== "evalResult") return;
      if (message.runId && message.runId !== runId) return;
      if (message.workflowUrl && message.workflowUrl !== workflow.url) return;
      const status = message.status ??
        (message.passed ? "passed" : "failed");
      finish({
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
        durationMs: typeof message.durationMs === "number"
          ? message.durationMs
          : 0,
      });
    };
    const onFailure = (event: Event) => {
      finish(
        infrastructureResult(
          workflow,
          runId,
          "socket",
          event.type === "close"
            ? "websocket closed before evalResult"
            : "websocket error before evalResult",
        ),
      );
    };
    const timer = setTimeout(() => {
      finish(
        infrastructureResult(
          workflow,
          runId,
          "result",
          `evalResult timed out after ${timeoutMs}ms`,
        ),
      );
    }, timeoutMs);
    socket.addEventListener("message", onMessage);
    socket.addEventListener("error", onFailure);
    socket.addEventListener("close", onFailure);
    const message: RunEvalMessage = {
      type: "runEval",
      runId,
      workflowUrl: workflow.url,
    };
    socket.send(JSON.stringify(message));
  });
}

function _escape(value: string): string {
  return value.replace(/&/g, "&amp;").replace(/</g, "&lt;")
    .replace(/>/g, "&gt;").replace(/"/g, "&quot;")
    .replace(/'/g, "&apos;");
}
