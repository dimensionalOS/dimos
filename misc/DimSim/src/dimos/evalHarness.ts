/**
 * EvalHarness — Browser-side eval orchestrator.
 *
 * Receives commands from the Deno eval runner (via WebSocket text messages),
 * runs eval workflow, scores objectDistance rubric at timeout, returns result.
 */

import {
  scoreObjectDistance,
  scoreRadiusContains,
  type SceneState,
  type ObjectDistanceCriteria,
  type RadiusContainsCriteria,
} from "./rubrics.ts";
import type { DimosBridge } from "./dimosBridge.ts";

export interface AgentPose { x: number; y: number; z: number; yaw: number; pitch: number; }
export interface StartPose { x?: number; y?: number; z?: number; yaw?: number; }

export interface SuccessCriteria {
  objectDistance?: ObjectDistanceCriteria;
  radiusContains?: RadiusContainsCriteria;
}

export interface Workflow {
  name: string;
  task: string;
  environment?: string;
  startPose?: StartPose;
  timeoutSec?: number;
  successCriteria?: SuccessCriteria;
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

export class EvalHarness {
  bridge: DimosBridge;
  getSceneState: () => SceneState;
  getAgentPose: () => AgentPose | null;
  channel: string;

  _workflow: Workflow | null = null;
  _startTime = 0;
  _timeoutTimer: ReturnType<typeof setTimeout> | null = null;
  _overlay: HTMLDivElement | null = null;

  constructor({ bridge, getSceneState, getAgentPose, channel }: EvalHarnessOptions) {
    this.bridge = bridge;
    this.getSceneState = getSceneState;
    this.getAgentPose = getAgentPose;
    this.channel = channel || "";
    this._hookBridgeMessages();
  }

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
    const evalTypes = new Set(["startWorkflow", "stopWorkflow", "loadEnv", "ping"]);
    ws.onmessage = (event: MessageEvent) => {
      if (typeof event.data === "string") {
        try {
          const cmd = JSON.parse(event.data);
          if (cmd.type && evalTypes.has(cmd.type)) {
            this._handleCommand(cmd);
            return;
          }
        } catch { /* not JSON, pass through */ }
        // Pass all non-eval text messages through to origOnMessage
        if (origOnMessage) (origOnMessage as (e: MessageEvent) => void).call(ws, event);
        return;
      }
      if (origOnMessage) (origOnMessage as (e: MessageEvent) => void).call(ws, event);
    };
  }

  _send(cmd: Record<string, any>): void {
    // Tag outgoing messages with channel for multi-page routing
    if (this.channel) cmd.channel = this.channel;
    this.bridge.sendCommand(cmd);
  }

  async _handleCommand(cmd: { type: string; channel?: string; workflow?: Workflow; [k: string]: any }): Promise<void> {
    // Channel filtering: if cmd has a channel and it doesn't match ours, ignore
    if (this.channel && cmd.channel && cmd.channel !== this.channel) return;
    console.log("[eval] command:", cmd.type);
    switch (cmd.type) {
      case "startWorkflow":
        await this._startWorkflow(cmd.workflow!);
        break;
      case "stopWorkflow":
        await this._stopWorkflow("runner-requested");
        break;
      case "loadEnv":
        // Scene is already loaded in --connect mode, just ack
        this._send({ type: "envReady", scene: cmd.scene });
        break;
      case "ping":
        this._send({ type: "pong", ts: Date.now() });
        break;
      default:
        break;
    }
  }

  async _startWorkflow(workflow: Workflow): Promise<void> {
    this._workflow = workflow;
    this._startTime = Date.now();

    console.log(`[eval] starting: ${workflow.name} — "${workflow.task}"`);

    if (workflow.startPose) {
      const p = workflow.startPose;
      const agent = window.__dimosAgent;
      if (agent) {
        agent.setPosition(p.x ?? 0, p.y ?? 0.5, p.z ?? 0);
        if (p.yaw !== undefined) agent.group.rotation.y = (p.yaw * Math.PI) / 180;
      }
    }

    const timeoutMs = (workflow.timeoutSec || 120) * 1000;
    this._timeoutTimer = setTimeout(() => this._stopWorkflow("timeout"), timeoutMs);

    this._showOverlay(workflow.task, workflow.timeoutSec || 120);
    this._send({ type: "workflowStarted", name: workflow.name });
  }

  async _stopWorkflow(reason: string): Promise<void> {
    if (!this._workflow) return;
    if (this._timeoutTimer) clearTimeout(this._timeoutTimer);
    this._timeoutTimer = null;

    console.log(`[eval] stopped: ${this._workflow.name} (${reason})`);

    const sceneState = this.getSceneState();
    const agentPose = this.getAgentPose();
    if (agentPose) {
      sceneState.agentPos = { x: agentPose.x, y: agentPose.y, z: agentPose.z };
    }

    const criteria = this._workflow.successCriteria || {};
    const scores: Record<string, any> = {};
    if (criteria.objectDistance) {
      scores.objectDistance = scoreObjectDistance(criteria.objectDistance, sceneState);
    }
    if (criteria.radiusContains) {
      scores.radiusContains = scoreRadiusContains(criteria.radiusContains, sceneState);
    }

    const pass = Object.values(scores).every((s: any) => s.pass !== false);
    const od = scores.objectDistance;
    this._showResult(pass, od ? od.details : reason);

    const result = {
      type: "workflowComplete",
      name: this._workflow.name,
      environment: this._workflow.environment,
      reason,
      durationMs: Date.now() - this._startTime,
      rubricScores: scores,
    };
    console.log("[eval] result:", result);
    this._send(result);
    this._workflow = null;
  }

  // -- UI overlay --------------------------------------------------------------

  _showOverlay(task: string, timeoutSec: number): void {
    if (this._overlay) this._overlay.remove();
    const el = document.createElement("div");
    el.style.cssText = "position:fixed;top:16px;left:50%;transform:translateX(-50%);z-index:99999;background:rgba(0,0,0,0.85);color:#fff;font:14px/1.5 monospace;padding:12px 24px;border-radius:10px;text-align:center;pointer-events:none;";
    const taskEl = document.createElement("div");
    taskEl.style.cssText = "color:#4fc3f7;font-size:16px;font-weight:bold;margin-bottom:4px;";
    taskEl.textContent = `EVAL: ${task}`;
    const timerEl = document.createElement("div");
    timerEl.style.cssText = "color:#aaa;font-size:13px;";
    el.appendChild(taskEl);
    el.appendChild(timerEl);
    document.body.appendChild(el);
    this._overlay = el;

    let remaining = timeoutSec;
    timerEl.textContent = `${remaining}s remaining`;
    const interval = setInterval(() => {
      remaining--;
      if (remaining <= 0 || !this._workflow) { clearInterval(interval); return; }
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
    if (this._timeoutTimer) clearTimeout(this._timeoutTimer);
    if (this._overlay) { this._overlay.remove(); this._overlay = null; }
  }
}
