/**
 * Eval Runner — Deno-side orchestrator.
 *
 * Walks `scenes/<env>/evals/*.js` to discover workflows, then for each one
 * opens a control WebSocket to the bridge, sends `{type:'runEval',
 * workflowUrl}`, and awaits the `{type:'evalResult', ...}` reply from the
 * browser-side harness.
 *
 * No JSON parsing.  No manifest.json.  The workflow file is the source of
 * truth — its `setup(ctx)` runs in the browser, its `success(ctx)` is
 * polled until passed or timeout, and the runner just collects results.
 */

import { resolve } from "@std/path";

export interface EvalResult {
  scene: string;
  workflow: string;
  workflowUrl: string;
  task: string;
  passed: boolean;
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

export interface RunEvalOptions {
  /** Control WebSocket URL (no `?ch=...`). */
  wsUrl: string;
  /** Absolute path to the scenes/ root. */
  scenesRoot: string;
  filterScene?: string;
  filterWorkflow?: string;
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
      continue; // no evals dir → no workflows for this scene
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
    console.log("[runner] no workflows match filter — nothing to do.");
    return [];
  }
  console.log(`[runner] running ${workflows.length} workflow(s)…`);

  const ws = await _connect(options.wsUrl);
  try {
    const results: EvalResult[] = [];
    for (const wf of workflows) {
      console.log(`[runner] → ${wf.scene}/${wf.workflow}`);
      const result = await _runOne(ws, wf);
      results.push(result);
      const tag = result.passed ? "PASS" : "FAIL";
      console.log(`[runner]   ${tag} (${result.durationMs}ms): ${result.reason}`);
    }
    return results;
  } finally {
    try { ws.close(); } catch { /* ignore */ }
  }
}

/** Parallel variant — one control WS per channel, workflows round-robin'd across them. */
export interface RunEvalsMultiPageOptions extends RunEvalOptions {
  /** Channel names from launchMultiPage (one per browser page). */
  channels: string[];
}

export async function runEvalsMultiPage(options: RunEvalsMultiPageOptions): Promise<EvalResult[]> {
  const workflows = collectWorkflows(options);
  if (workflows.length === 0 || options.channels.length === 0) return [];

  // Open one socket per channel.
  // Two query params:
  //   channel=<name>  → routes the WS to that channel's bridge state
  //   ch=control      → marks it as a control (not sensor) socket so text
  //                     frames (the `runEval` JSON) are processed, not dropped
  const sockets = await Promise.all(
    options.channels.map((ch) =>
      _connect(`${options.wsUrl}/?channel=${encodeURIComponent(ch)}&ch=control`),
    ),
  );

  // Round-robin workflows across sockets.
  const queues: WorkflowEntry[][] = sockets.map(() => []);
  workflows.forEach((wf, i) => queues[i % queues.length].push(wf));

  try {
    const all = await Promise.all(
      sockets.map(async (ws, i) => {
        const out: EvalResult[] = [];
        for (const wf of queues[i]) {
          console.log(`[runner:${options.channels[i]}] → ${wf.scene}/${wf.workflow}`);
          out.push(await _runOne(ws, wf));
        }
        return out;
      }),
    );
    return all.flat();
  } finally {
    for (const ws of sockets) {
      try { ws.close(); } catch { /* ignore */ }
    }
  }
}

/** JUnit-style XML emitter for CI consumption. */
export function toJunitXml(results: EvalResult[]): string {
  const lines: string[] = [];
  lines.push('<?xml version="1.0" encoding="UTF-8"?>');
  const failures = results.filter((r) => !r.passed).length;
  lines.push(`<testsuite name="dimsim-evals" tests="${results.length}" failures="${failures}">`);
  for (const r of results) {
    const name = `${r.scene}/${r.workflow}`;
    const time = (r.durationMs / 1000).toFixed(3);
    if (r.passed) {
      lines.push(`  <testcase name="${name}" time="${time}"/>`);
    } else {
      lines.push(`  <testcase name="${name}" time="${time}">`);
      lines.push(`    <failure message="${_escape(r.reason)}"/>`);
      lines.push(`  </testcase>`);
    }
  }
  lines.push("</testsuite>");
  return lines.join("\n");
}

// ── Internals ────────────────────────────────────────────────────────────────

function _connect(wsUrl: string): Promise<WebSocket> {
  // Force the control channel so eval text messages route correctly.
  const url = wsUrl.includes("?") ? wsUrl : `${wsUrl}/?ch=control`;
  const ws = new WebSocket(url);
  return new Promise((resolve, reject) => {
    ws.addEventListener("open", () => resolve(ws), { once: true });
    ws.addEventListener("error", (e) => reject(e), { once: true });
  });
}

function _runOne(ws: WebSocket, wf: WorkflowEntry): Promise<EvalResult> {
  return new Promise((resolve) => {
    const cleanup = () => {
      ws.removeEventListener("message", onMessage);
      ws.removeEventListener("error", onFail);
      ws.removeEventListener("close", onFail);
    };
    const onMessage = (event: MessageEvent) => {
      if (typeof event.data !== "string") return;
      let msg: any;
      try { msg = JSON.parse(event.data); } catch { return; }
      if (msg.type !== "evalResult") return;
      if (msg.workflowUrl && msg.workflowUrl !== wf.url) return;
      cleanup();
      resolve({
        scene: wf.scene,
        workflow: wf.workflow,
        workflowUrl: wf.url,
        task: msg.task ?? "",
        passed: !!msg.passed,
        reason: msg.reason ?? (msg.passed ? "ok" : "fail"),
        score: typeof msg.score === "number" ? msg.score : null,
        durationMs: msg.durationMs ?? 0,
      });
    };
    // If the socket closes or errors before the result lands, fail the eval
    // instead of hanging the entire runEvals call forever.
    const onFail = (event: Event) => {
      cleanup();
      const reason = event.type === "close"
        ? "websocket closed before evalResult"
        : "websocket error before evalResult";
      resolve({
        scene: wf.scene,
        workflow: wf.workflow,
        workflowUrl: wf.url,
        task: "",
        passed: false,
        reason,
        score: null,
        durationMs: 0,
      });
    };
    ws.addEventListener("message", onMessage);
    ws.addEventListener("error", onFail);
    ws.addEventListener("close", onFail);
    ws.send(JSON.stringify({ type: "runEval", workflowUrl: wf.url }));
  });
}

function _escape(s: string): string {
  return s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;").replace(/'/g, "&apos;");
}
