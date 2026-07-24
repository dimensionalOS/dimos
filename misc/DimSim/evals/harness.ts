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

/// <reference lib="dom" />

import {
  type SceneState, type AssetEntry,
  type ObjectDistanceOpts, type RadiusContainsOpts,
  findAsset, dist, objectDistance, radiusContains,
} from "./rubrics.ts";
import type { DimosBridge } from "../src/bridge.ts";
import type {
  EvalAbortMessage,
  EvalFailureStage,
  EvalReadyMessage,
  EvalResultMessage,
  EvalStartMessage,
  RunEvalMessage,
} from "./protocol.ts";
import { isFiniteStartPose } from "./protocol.ts";

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

export type EvalResultMsg = EvalResultMessage;

export interface EvalHarnessOptions {
  bridge: DimosBridge;
  getSceneState: () => SceneState;
  getAgentPose: () => AgentPose | null;
  channel?: string;
}

declare global {
  interface Window { __dimosAgent?: any; }
}

interface ActiveCommand {
  runId: string;
  workflowUrl: string;
  agent: boolean;
}

interface PendingAgentEval {
  runId: string;
  workflowUrl: string;
  workflow: EvalWorkflow;
  resolve: (message: EvalResultMsg) => void;
  timer: ReturnType<typeof setTimeout> | null;
  started: boolean;
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
  _command: ActiveCommand | null = null;
  _pendingAgentEval: PendingAgentEval | null = null;
  _earlyAborts = new Map<string, EvalAbortMessage>();

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
    const evalTypes = new Set(["runEval", "evalStart", "evalAbort"]);
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
        await this._loadAndRunWorkflowFile(cmd as RunEvalMessage);
        break;
      case "evalStart":
        this._startAgentEval(cmd as EvalStartMessage);
        break;
      case "evalAbort":
        this._abortAgentEval(cmd as EvalAbortMessage);
        break;
    }
  }

  /**
   * WS-driven entry: dynamic-import a workflow file.  The file's top-level
   * `await runEval({...})` (via the `@dimsim/eval` import map) calls
   * `this.runEval(workflow)` and sends the result WS message itself.  We
   * just await the import — when it resolves the eval is done.
   */
  async _loadAndRunWorkflowFile(cmd: RunEvalMessage): Promise<void> {
    const workflowUrl = cmd.workflowUrl;
    const runId = cmd.runId || crypto.randomUUID();
    this._command = { runId, workflowUrl, agent: cmd.agent === true };
    try {
      const cacheBust = `?t=${Date.now()}`;
      await import(/* @vite-ignore */ workflowUrl + cacheBust);
    } catch (e: any) {
      console.error("[eval] failed to import %s:", workflowUrl, e);
      this._activeUrl = null;
      this._fail(
        runId,
        workflowUrl,
        "",
        "",
        `import failed: ${e?.message ?? e}`,
        0,
        "import",
      );
    } finally {
      this._earlyAborts.delete(runId);
      if (this._command?.runId === runId) this._command = null;
    }
  }

  // ── Public entry point (called by workflow files via @dimsim/eval) ─────────

  /**
   * Run a workflow object end-to-end.  Workflow files do:
   *
   *     import { runEval } from '@dimsim/eval';
   *     await runEval({ scene, task, success, … });
   *
   * That import resolves via the index.html importmap to the pinned
   * `/assets/dimsim-eval.js` chunk (this module), whose exported `runEval`
   * delegates to this method on the engine-registered singleton.  Result is
   * both returned to the caller AND sent over WS as `{type:'evalResult'}`
   * for the Deno runner.
  */
  async runEval(workflow: EvalWorkflow): Promise<EvalResultMsg> {
    const command = this._command ?? {
      runId: crypto.randomUUID(),
      workflowUrl: "",
      agent: false,
    };
    if (
      !workflow ||
      typeof workflow.scene !== "string" ||
      typeof workflow.task !== "string" ||
      workflow.task.length === 0 ||
      typeof workflow.success !== "function"
    ) {
      const msg = "runEval(workflow) requires { scene, task, success() }";
      console.error("[eval] %s", msg);
      return this._fail(
        command.runId,
        command.workflowUrl,
        workflow?.scene ?? "",
        workflow?.task ?? "",
        msg,
        0,
        "configuration",
      );
    }
    const tag = `${workflow.scene ?? "?"}/${workflow.task}`;
    if (this._activeUrl) {
      const err = `another eval is already running: ${this._activeUrl}`;
      console.warn("[eval] %s", err);
      return this._fail(
        command.runId,
        command.workflowUrl,
        workflow.scene,
        workflow.task,
        err,
        0,
        "configuration",
      );
    }
    this._activeUrl = tag;

    if (command.agent) {
      return await this._prepareAgentEval(command, workflow);
    }

    console.log("[eval] running: %s", tag);
    this._showOverlay(workflow.task, workflow.timeoutSec ?? 120);

    const start = Date.now();
    const timeoutMs = (workflow.timeoutSec ?? 120) * 1000;
    const ctx = this._makeContext();

    if (workflow.startPose) ctx.setAgentPose(workflow.startPose);
    if (workflow.setup) {
      try { await workflow.setup(ctx); }
      catch (e: any) {
        const reason = `setup() threw: ${e?.message ?? e}`;
        console.error("[eval] %s", reason);
        this._activeUrl = null;
        return this._fail(
          command.runId,
          command.workflowUrl,
          workflow.scene,
          workflow.task,
          reason,
          Date.now() - start,
          "setup",
        );
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
          this._finish(
            command.runId,
            command.workflowUrl,
            workflow,
            true,
            result,
            elapsed,
            resolve,
          );
          return;
        }
        if (elapsed >= timeoutMs) {
          this._finish(
            command.runId,
            command.workflowUrl,
            workflow,
            false,
            { ...result, passed: false, reason: result.reason ?? "timeout" },
            elapsed,
            resolve,
          );
          return;
        }
        setTimeout(tick, 250);
      };
      tick();
    });
  }

  // ── Internals ──────────────────────────────────────────────────────────────

  async _prepareAgentEval(
    command: ActiveCommand,
    workflow: EvalWorkflow,
  ): Promise<EvalResultMsg> {
    const alreadyAborted = this._takeEarlyAbort(command, workflow);
    if (alreadyAborted) return alreadyAborted;

    const timeoutSec = workflow.timeoutSec ?? 120;
    if (!Number.isFinite(timeoutSec) || timeoutSec <= 0) {
      this._activeUrl = null;
      return this._fail(
        command.runId,
        command.workflowUrl,
        workflow.scene,
        workflow.task,
        "agent eval requires a finite positive timeoutSec",
        0,
        "configuration",
      );
    }
    const startPose = workflow.startPose;
    if (!isFiniteStartPose(startPose)) {
      this._activeUrl = null;
      return this._fail(
        command.runId,
        command.workflowUrl,
        workflow.scene,
        workflow.task,
        "agent eval startPose requires finite x, y, z, and yaw",
        0,
        "configuration",
      );
    }

    const setupStart = Date.now();
    if (workflow.setup) {
      try {
        await workflow.setup(this._makeContext());
      } catch (e: any) {
        const aborted = this._takeEarlyAbort(command, workflow);
        if (aborted) return aborted;
        const reason = `setup() threw: ${e?.message ?? e}`;
        this._activeUrl = null;
        return this._fail(
          command.runId,
          command.workflowUrl,
          workflow.scene,
          workflow.task,
          reason,
          Date.now() - setupStart,
          "setup",
        );
      }
    }

    const abortedAfterSetup = this._takeEarlyAbort(command, workflow);
    if (abortedAfterSetup) return abortedAfterSetup;

    // Evaluate against the declared authoritative start pose without moving the
    // browser-only avatar.  A task that is already satisfied cannot measure
    // agent behavior and is rejected before reset or model dispatch.
    let initial: EvalSuccess;
    try {
      initial = workflow.success(this._makeContext(startPose));
    } catch (e: any) {
      this._activeUrl = null;
      return this._fail(
        command.runId,
        command.workflowUrl,
        workflow.scene,
        workflow.task,
        `initial success() threw: ${e?.message ?? e}`,
        0,
        "rubric",
      );
    }
    if (initial.passed) {
      this._activeUrl = null;
      return this._fail(
        command.runId,
        command.workflowUrl,
        workflow.scene,
        workflow.task,
        "agent eval rubric is already satisfied at startPose",
        0,
        "initialRubric",
      );
    }

    return await new Promise<EvalResultMsg>((resolve) => {
      this._pendingAgentEval = {
        runId: command.runId,
        workflowUrl: command.workflowUrl,
        workflow,
        resolve,
        timer: null,
        started: false,
      };
      const ready: EvalReadyMessage = {
        type: "evalReady",
        runId: command.runId,
        workflowUrl: command.workflowUrl,
        scene: workflow.scene,
        task: workflow.task,
        timeoutMs: timeoutSec * 1000,
        startPose,
      };
      console.log("[eval] ready and waiting for authoritative reset: %s", this._activeUrl);
      this._send(ready);
    });
  }

  _startAgentEval(message: EvalStartMessage): void {
    const pending = this._pendingAgentEval;
    if (!pending || pending.runId !== message.runId || pending.started) return;
    pending.started = true;

    const workflow = pending.workflow;
    const timeoutMs = (workflow.timeoutSec ?? 120) * 1000;
    const start = Date.now();
    console.log("[eval] agent scoring started: %s", this._activeUrl);
    this._showOverlay(workflow.task, workflow.timeoutSec ?? 120);

    const tick = () => {
      if (this._pendingAgentEval !== pending) return;
      const elapsed = Date.now() - start;
      let result: EvalSuccess;
      try {
        result = workflow.success(this._makeContext());
      } catch (e: any) {
        const reason = `success() threw: ${e?.message ?? e}`;
        this._pendingAgentEval = null;
        this._activeUrl = null;
        pending.resolve(
          this._fail(
            pending.runId,
            pending.workflowUrl,
            workflow.scene,
            workflow.task,
            reason,
            elapsed,
            "rubric",
          ),
        );
        return;
      }
      if (result.passed) {
        this._finish(
          pending.runId,
          pending.workflowUrl,
          workflow,
          true,
          result,
          elapsed,
          pending.resolve,
        );
        return;
      }
      if (elapsed >= timeoutMs) {
        this._finish(
          pending.runId,
          pending.workflowUrl,
          workflow,
          false,
          { ...result, passed: false, reason: result.reason ?? "timeout" },
          elapsed,
          pending.resolve,
        );
        return;
      }
      pending.timer = setTimeout(tick, 250);
    };
    tick();
  }

  _abortAgentEval(message: EvalAbortMessage): void {
    const pending = this._pendingAgentEval;
    if (!pending || pending.runId !== message.runId) {
      if (this._command?.runId === message.runId) {
        this._earlyAborts.set(message.runId, message);
      }
      return;
    }
    if (pending.timer) clearTimeout(pending.timer);
    this._pendingAgentEval = null;
    this._activeUrl = null;
    if (this._overlay) {
      this._overlay.remove();
      this._overlay = null;
    }
    pending.resolve({
      type: "evalResult",
      runId: pending.runId,
      workflowUrl: pending.workflowUrl,
      scene: pending.workflow.scene,
      task: pending.workflow.task,
      passed: false,
      status: "error",
      failureStage: message.failureStage,
      reason: message.reason,
      durationMs: 0,
    });
  }

  _takeEarlyAbort(
    command: ActiveCommand,
    workflow: EvalWorkflow,
  ): EvalResultMsg | null {
    const abort = this._earlyAborts.get(command.runId);
    if (!abort) return null;
    this._earlyAborts.delete(command.runId);
    this._activeUrl = null;
    return {
      type: "evalResult",
      runId: command.runId,
      workflowUrl: command.workflowUrl,
      scene: workflow.scene,
      task: workflow.task,
      passed: false,
      status: "error",
      failureStage: abort.failureStage,
      reason: abort.reason,
      durationMs: 0,
    };
  }

  _makeContext(agentPosOverride?: { x: number; y: number; z: number }): EvalContext {
    const sceneState = this.getSceneState();
    const pose = this.getAgentPose();
    const agentPos = agentPosOverride
      ? {
        x: agentPosOverride.x,
        y: agentPosOverride.y,
        z: agentPosOverride.z,
      }
      : pose
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
    runId: string,
    workflowUrl: string,
    wf: EvalWorkflow,
    passed: boolean,
    result: EvalSuccess, durationMs: number,
    resolve: (msg: EvalResultMsg) => void,
  ): void {
    const pending = this._pendingAgentEval;
    if (pending?.timer) clearTimeout(pending.timer);
    const msg: EvalResultMsg = {
      type: "evalResult",
      runId,
      workflowUrl,
      scene: wf.scene,
      task: wf.task,
      passed,
      status: passed ? "passed" : "failed",
      reason: result.reason,
      score: result.score,
      durationMs,
    };
    console.log("[eval] %s (%dms): %s", passed ? "PASS" : "FAIL", durationMs, result.reason ?? "");
    this._showResult(passed, result.reason ?? (passed ? "ok" : "fail"));
    this._send(msg);
    this._pendingAgentEval = null;
    this._activeUrl = null;
    resolve(msg);
  }

  _fail(
    runId: string,
    workflowUrl: string,
    scene: string,
    task: string,
    reason: string,
    durationMs = 0,
    failureStage?: EvalFailureStage,
  ): EvalResultMsg {
    const msg: EvalResultMsg = {
      type: "evalResult",
      runId,
      workflowUrl,
      scene,
      task,
      passed: false,
      status: failureStage ? "error" : "failed",
      failureStage,
      reason,
      durationMs,
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
}
