/// <reference lib="deno.ns" />
/**
 * Deno-side `@dimsim/eval` runEval — dispatch a workflow file to the
 * browser via the bridge instead of running it locally.
 *
 * A workflow file under scenes/<env>/evals/<name>.js does:
 *
 *     import { runEval } from '@dimsim/eval';
 *     await runEval({ scene, task, success, ... });
 *
 * In the browser, that import resolves (via the index.html importmap)
 * to the bundled EvalHarness chunk and runs the eval in-place.
 *
 * In Deno, scenes/deno.json maps `@dimsim/eval` here.  We don't have a
 * THREE.js scene, agent, or Rapier — we just open a control WebSocket
 * to a running bridge, ship `{type:'runEval', workflowUrl}`, and wait
 * for `{type:'evalResult'}`.  The browser imports the same workflow
 * file URL and runs the success/setup callbacks for real.
 *
 * Net effect: `deno run -A scenes/<env>/evals/<name>.js` is a one-liner
 * shortcut for `dimsim eval <env>/<name>`, both end at the same
 * EvalHarness in the open browser.
 */

import { fromFileUrl } from "@std/path";

interface DenoEvalResult {
  type: "evalResult";
  workflowUrl: string;
  scene: string;
  task: string;
  passed: boolean;
  reason?: string;
  score?: number;
  durationMs: number;
}

/** Find the "/scenes/..." segment in the entry-point file path. */
function _resolveWorkflowUrl(): string {
  const main = Deno.mainModule;
  if (!main.startsWith("file://")) {
    throw new Error(
      `@dimsim/eval: can only infer workflow URL from a 'deno run <file>' invocation, got ${main}`,
    );
  }
  const abs = fromFileUrl(main);
  const i = abs.indexOf("/scenes/");
  if (i === -1) {
    throw new Error(
      `@dimsim/eval: workflow file must live under a 'scenes/' directory; got ${abs}`,
    );
  }
  return abs.slice(i); // e.g. "/scenes/apartment/evals/go-to-couch.js"
}

/** Open the control WebSocket, race resolve / error / 5s timeout. */
function _connect(wsUrl: string): Promise<WebSocket> {
  return new Promise((resolve, reject) => {
    const ws = new WebSocket(wsUrl);
    const t = setTimeout(() => reject(new Error(`@dimsim/eval: timed out connecting to ${wsUrl}`)), 5000);
    ws.addEventListener("open", () => { clearTimeout(t); resolve(ws); }, { once: true });
    ws.addEventListener("error", (e) => { clearTimeout(t); reject(e); }, { once: true });
  });
}

/**
 * Public entry — same name + same call shape as the browser export, so
 * workflow files don't change between runtimes.  The `workflow` object
 * is forwarded only as a logging convenience here; the browser is what
 * actually runs `setup` / `success` (it re-imports the file from URL).
 */
export async function runEval(workflow: { scene?: string; task?: string }): Promise<DenoEvalResult> {
  const workflowUrl = _resolveWorkflowUrl();
  const port = parseInt(Deno.env.get("DIMSIM_PORT") || "8090");
  const wsUrl = `ws://localhost:${port}/?ch=control`;

  console.log(`[eval] dispatching ${workflowUrl} → ${wsUrl}`);
  if (workflow?.task) console.log(`[eval] task: ${workflow.task}`);

  let ws: WebSocket;
  try {
    ws = await _connect(wsUrl);
  } catch (e: any) {
    console.error(`[eval] failed to connect to bridge — is dimsim running? (${e?.message ?? e})`);
    Deno.exit(2);
  }

  try {
    const result = await new Promise<DenoEvalResult>((resolve) => {
      ws.addEventListener("message", (event) => {
        if (typeof event.data !== "string") return;
        let msg: any;
        try { msg = JSON.parse(event.data); } catch { return; }
        if (msg.type !== "evalResult") return;
        if (msg.workflowUrl && msg.workflowUrl !== workflowUrl) return;
        resolve(msg as DenoEvalResult);
      });
      ws.send(JSON.stringify({ type: "runEval", workflowUrl }));
    });

    const tag = result.passed ? "\x1b[32mPASS\x1b[0m" : "\x1b[31mFAIL\x1b[0m";
    console.log(`[eval] ${tag} (${result.durationMs}ms): ${result.reason ?? ""}`);
    if (!result.passed) Deno.exit(1);
    return result;
  } finally {
    try { ws.close(); } catch { /* ignore */ }
  }
}
