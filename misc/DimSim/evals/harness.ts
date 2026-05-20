/**
 * EvalHarness — browser-side runner for JS-native eval workflows.
 *
 * A workflow is a JS module under `scenes/<env>/evals/<name>.js` whose
 * default export shapes like:
 *
 *     export default {
 *       scene: 'apartment',
 *       task:  'Go to the couch',
 *       timeoutSec: 30,
 *       startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },   // optional sugar
 *       setup:   async (ctx) => { … },               // optional
 *       success: (ctx) => ({ passed, reason?, score? }),
 *     };
 *
 * The Deno runner sends one `{type:'runEval', workflowUrl, channel?}` WS
 * message; this class dynamic-imports the module, runs `setup(ctx)` once,
 * then polls `success(ctx)` every 250 ms until passed or timeout, and
 * replies with `{type:'evalResult', …}`.  No JSON criteria, no runner-side
 * orchestration — the workflow file is the source of truth.
 */

import {
  type SceneState, type AssetEntry, type EvalSuccess as _EvalSuccess,
  type ObjectDistanceOpts, type RadiusContainsOpts,
  findAsset, dist, objectDistance, radiusContains,
} from "./rubrics.ts";
import type { DimosBridge } from "../bridge.ts";

export interface AgentPose { x: number; y: number; z: number; yaw: number; pitch: number; }
export interface StartPose { x?: number; y?: number; z?: number; yaw?: number; }

/** Shape returned by `workflow.success(ctx)`. */
export interface EvalSuccess {
  passed: boolean;
  reason?: string;
  score?: number;
}

/** Context passed to `workflow.setup(ctx)` and `workflow.success(ctx)`. */
export interface EvalContext {
  agent: any;
  agentPos: { x: number; y: number; z: number };
  sceneState: SceneState;
  setAgentPose: (p: StartPose) => void;
  findAsset: (q: string) => AssetEntry | null;
  dist: (a: { x: number; y: number; z: number }, b: { x: number; y: number; z: number }) => number;
  /** Pre-bound high-level rubric helpers — `ctx.rubrics.objectDistance({...})` etc. */
  rubrics: {
    objectDistance: (opts: ObjectDistanceOpts) => EvalSuccess;
    radiusContains: (opts: RadiusContainsOpts) => EvalSuccess;
  };
}

/** Default-export shape of a workflow file. */
export interface EvalWorkflow {
  scene: string;
  task: string;
  timeoutSec?: number;
  startPose?: StartPose;
  setup?: (ctx: EvalContext) => void | Promise<void>;
  success: (ctx: EvalContext) => EvalSuccess;
}

export interface EvalResultMsg {
  type: "evalResult";
  workflowUrl: string;
  scene: string;
  task: string;
  passed: boolean;
  reason?: string;
  score?: number;
  durationMs: number;
  channel?: string;
}

export interface EvalHarnessOptions {
  bridge: DimosBridge;
  getSceneState: () => SceneState;
  getAgentPose: () => AgentPose | null;
  channel?: string;
}

declare global {
  interface Window { __dimosAgent?: any; }
}

// ── Singleton registration ──────────────────────────────────────────────────
//
// Workflow files import `runEval` from `@dimsim/eval`.  The importmap in
// index.html points that bare specifier at this very chunk's bundled
// filename (pinned by vite.config.js → `dist/assets/dimsim-eval.js`), so
// the workflow ends up importing this same module — which means it sees
// the `_instance` set below by engine.js after EvalHarness construction.

let _instance: EvalHarness | null = null;
let _readyResolvers: Array<() => void> = [];

/** engine.js calls this once the harness is wired up. */
export function setEvalHarness(h: EvalHarness): void {
  _instance = h;
  const r = _readyResolvers;
  _readyResolvers = [];
  for (const fn of r) fn();
}

async function _waitForInstance(): Promise<EvalHarness> {
  if (_instance) return _instance;
  await new Promise<void>((resolve) => _readyResolvers.push(resolve));
  return _instance!;
}

/**
 * Public entry — what workflow files call after importing from
 * `@dimsim/eval`.  Resolves when the workflow finishes (passed, failed,
 * or timed out); also sends a `{type:'evalResult'}` WS message for the
 * Deno runner along the way.
 */
export async function runEval(workflow: EvalWorkflow): Promise<EvalResultMsg> {
  const h = await _waitForInstance();
  return h.runEval(workflow);
}

// ────────────────────────────────────────────────────────────────────────────

export class EvalHarness {
  bridge: DimosBridge;
  getSceneState: () => SceneState;
  getAgentPose: () => AgentPose | null;
  channel: string;

  _activeUrl: string | null = null;
  _overlay: HTMLDivElement | null = null;

  constructor({ bridge, getSceneState, getAgentPose, channel }: EvalHarnessOptions) {
    this.bridge = bridge;
    this.getSceneState = getSceneState;
    this.getAgentPose = getAgentPose;
    this.channel = channel || "";
    this._hookBridgeMessages();
  }

  // ── WS plumbing ────────────────────────────────────────────────────────────

  _hookBridgeMessages(): void {
    const origConnect = this.bridge.connect.bind(this.bridge);
    this.bridge.connect = () => {
      origConnect();
      setTimeout(() => {
        const ws = this.bridge.ws;
        if (ws) this._patchWsOnMessage(ws);
      }, 100);
    };
    const ws = this.bridge.ws;
    if (ws) this._patchWsOnMessage(ws);
  }

  _patchWsOnMessage(ws: WebSocket): void {
    const origOnMessage = ws.onmessage;
    const evalTypes = new Set(["runEval", "ping"]);
    ws.onmessage = (event: MessageEvent) => {
      if (typeof event.data === "string") {
        try {
          const cmd = JSON.parse(event.data);
          if (cmd.type && evalTypes.has(cmd.type)) {
            this._handleCommand(cmd);
            return;
          }
        } catch { /* not JSON */ }
        if (origOnMessage) (origOnMessage as (e: MessageEvent) => void).call(ws, event);
        return;
      }
      if (origOnMessage) (origOnMessage as (e: MessageEvent) => void).call(ws, event);
    };
  }

  _send(msg: Record<string, any>): void {
    if (this.channel) msg.channel = this.channel;
    this.bridge.sendCommand(msg);
  }

  async _handleCommand(cmd: { type: string; channel?: string; [k: string]: any }): Promise<void> {
    if (this.channel && cmd.channel && cmd.channel !== this.channel) return;
    switch (cmd.type) {
      case "runEval":
        await this._loadAndRunWorkflowFile(cmd.workflowUrl);
        break;
      case "ping":
        this._send({ type: "pong", ts: Date.now() });
        break;
    }
  }

  /**
   * WS-driven entry: dynamic-import a workflow file.  The file's top-level
   * `await runEval({...})` (via the `@dimsim/eval` import map) calls
   * `this.runEval(workflow)` and sends the result WS message itself.  We
   * just await the import — when it resolves the eval is done.
   */
  async _loadAndRunWorkflowFile(workflowUrl: string): Promise<void> {
    try {
      const cacheBust = `?t=${Date.now()}`;
      await import(/* @vite-ignore */ workflowUrl + cacheBust);
    } catch (e: any) {
      console.error("[eval] failed to import %s:", workflowUrl, e);
      this._send({
        type: "evalResult", workflowUrl, scene: "", task: "",
        passed: false, reason: `import failed: ${e?.message ?? e}`,
        durationMs: 0,
      });
    }
  }

  // ── Public entry point (called by workflow files via @dimsim/eval) ─────────

  /**
   * Run a workflow object end-to-end.  Workflow files do:
   *
   *     import { runEval } from '@dimsim/eval';
   *     await runEval({ scene, task, success, … });
   *
   * That import resolves to public/_dimsim/eval-api.js which delegates to
   * this method via `window.__dimsim.eval.runEval`.  Result is both
   * returned to the caller AND sent over WS as `{type:'evalResult'}` for
   * the Deno runner.
   */
  async runEval(workflow: EvalWorkflow): Promise<EvalResultMsg> {
    if (!workflow || typeof workflow.success !== "function") {
      const msg = "runEval(workflow) requires { scene, task, success() }";
      console.error(`[eval] ${msg}`);
      return this._fail("", "", "", msg);
    }
    const tag = `${workflow.scene ?? "?"}/${workflow.task}`;
    if (this._activeUrl) {
      const err = `another eval is already running: ${this._activeUrl}`;
      console.warn(`[eval] ${err}`);
      return this._fail("", workflow.scene, workflow.task, err);
    }
    this._activeUrl = tag;

    console.log(`[eval] running: ${tag}`);
    this._showOverlay(workflow.task, workflow.timeoutSec ?? 120);

    const start = Date.now();
    const timeoutMs = (workflow.timeoutSec ?? 120) * 1000;
    const ctx = this._makeContext();

    if (workflow.startPose) ctx.setAgentPose(workflow.startPose);
    if (workflow.setup) {
      try { await workflow.setup(ctx); }
      catch (e: any) {
        const reason = `setup() threw: ${e?.message ?? e}`;
        console.error(`[eval] ${reason}`);
        this._activeUrl = null;
        return this._fail("", workflow.scene, workflow.task, reason, Date.now() - start);
      }
    }

    return new Promise<EvalResultMsg>((resolve) => {
      const tick = () => {
        const elapsed = Date.now() - start;
        let result: EvalSuccess;
        try {
          result = workflow.success(this._makeContext());
        } catch (e: any) {
          result = { passed: false, reason: `success() threw: ${e?.message ?? e}` };
        }
        if (result.passed) {
          this._finish(workflow, true, result, elapsed, resolve);
          return;
        }
        if (elapsed >= timeoutMs) {
          this._finish(workflow, false, { passed: false, ...result, reason: result.reason ?? "timeout" }, elapsed, resolve);
          return;
        }
        setTimeout(tick, 250);
      };
      tick();
    });
  }

  // ── Internals ──────────────────────────────────────────────────────────────

  _makeContext(): EvalContext {
    const sceneState = this.getSceneState();
    const pose = this.getAgentPose();
    const agentPos = pose
      ? { x: pose.x, y: pose.y, z: pose.z }
      : { x: 0, y: 0, z: 0 };
    sceneState.agentPos = agentPos;
    const ctxLite = { agentPos, sceneState };
    return {
      agent: window.__dimosAgent,
      agentPos,
      sceneState,
      setAgentPose: (p) => {
        const a = window.__dimosAgent;
        if (!a) return;
        a.setPosition(p.x ?? 0, p.y ?? 0.5, p.z ?? 0);
        if (p.yaw !== undefined && a.group) a.group.rotation.y = (p.yaw * Math.PI) / 180;
      },
      findAsset: (q) => findAsset(q, sceneState),
      dist,
      rubrics: {
        objectDistance: (opts) => objectDistance(ctxLite, opts),
        radiusContains: (opts) => radiusContains(ctxLite, opts),
      },
    };
  }

  _finish(
    wf: EvalWorkflow, passed: boolean,
    result: EvalSuccess, durationMs: number,
    resolve: (msg: EvalResultMsg) => void,
  ): void {
    const msg: EvalResultMsg = {
      type: "evalResult",
      workflowUrl: "",
      scene: wf.scene,
      task: wf.task,
      passed,
      reason: result.reason,
      score: result.score,
      durationMs,
    };
    console.log(`[eval] ${passed ? "PASS" : "FAIL"} (${durationMs}ms): ${result.reason ?? ""}`);
    this._showResult(passed, result.reason ?? (passed ? "ok" : "fail"));
    this._send(msg);
    this._activeUrl = null;
    resolve(msg);
  }

  _fail(workflowUrl: string, scene: string, task: string, reason: string, durationMs = 0): EvalResultMsg {
    const msg: EvalResultMsg = {
      type: "evalResult", workflowUrl, scene, task,
      passed: false, reason, durationMs,
    };
    this._send(msg);
    return msg;
  }

  // ── UI overlay ─────────────────────────────────────────────────────────────

  _showOverlay(task: string, timeoutSec: number): void {
    if (this._overlay) this._overlay.remove();
    const el = document.createElement("div");
    el.style.cssText = "position:fixed;top:16px;left:50%;transform:translateX(-50%);z-index:99999;background:rgba(0,0,0,0.85);color:#fff;font:14px/1.5 monospace;padding:12px 24px;border-radius:10px;text-align:center;pointer-events:none;";
    const taskEl = document.createElement("div");
    taskEl.style.cssText = "color:#4fc3f7;font-size:16px;font-weight:bold;margin-bottom:4px;";
    taskEl.textContent = `EVAL: ${task}`;
    const timerEl = document.createElement("div");
    timerEl.style.cssText = "color:#aaa;font-size:13px;";
    el.appendChild(taskEl); el.appendChild(timerEl);
    document.body.appendChild(el);
    this._overlay = el;

    let remaining = timeoutSec;
    timerEl.textContent = `${remaining}s remaining`;
    const interval = setInterval(() => {
      remaining--;
      if (remaining <= 0 || !this._activeUrl) { clearInterval(interval); return; }
      timerEl.textContent = `${remaining}s remaining`;
    }, 1000);
  }

  _showResult(pass: boolean, details: string): void {
    if (this._overlay) this._overlay.remove();
    const el = document.createElement("div");
    const bg = pass ? "rgba(46,125,50,0.9)" : "rgba(198,40,40,0.9)";
    el.style.cssText = `position:fixed;top:16px;left:50%;transform:translateX(-50%);z-index:99999;background:${bg};color:#fff;font:14px/1.5 monospace;padding:12px 24px;border-radius:10px;text-align:center;pointer-events:none;`;
    el.textContent = `${pass ? "PASS" : "FAIL"}: ${details}`;
    document.body.appendChild(el);
    this._overlay = el;
    setTimeout(() => { if (this._overlay === el) { el.remove(); this._overlay = null; } }, 5000);
  }

  dispose(): void {
    if (this._overlay) { this._overlay.remove(); this._overlay = null; }
  }
}
