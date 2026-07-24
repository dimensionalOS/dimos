/**
 * Correlated messages used by the Deno eval runner, bridge, and browser
 * harness.  `runId` is mandatory for new callers; the browser harness still
 * accepts legacy `runEval` messages so direct workflow execution keeps working.
 */

export type EvalStatus = "passed" | "failed" | "error";

export type EvalFailureStage =
  | "configuration"
  | "connection"
  | "browserReady"
  | "reset"
  | "mcp"
  | "result"
  | "socket"
  | "import"
  | "setup"
  | "initialRubric"
  | "rubric";

/** Workflow start poses use Three.js coordinates and yaw in degrees. */
export interface EvalStartPose {
  x: number;
  y: number;
  z: number;
  yaw: number;
}

export interface RunEvalMessage {
  type: "runEval";
  runId: string;
  workflowUrl: string;
  agent?: boolean;
  channel?: string;
}

export interface EvalReadyMessage {
  type: "evalReady";
  runId: string;
  workflowUrl: string;
  scene: string;
  task: string;
  timeoutMs: number;
  startPose: EvalStartPose;
  channel?: string;
}

export interface EvalResetMessage {
  type: "evalReset";
  runId: string;
  startPose: EvalStartPose;
  channel?: string;
}

export interface ResetAckMessage {
  type: "resetAck";
  runId: string;
  ok: boolean;
  pose?: EvalStartPose;
  reason?: string;
  channel?: string;
}

export interface EvalStartMessage {
  type: "evalStart";
  runId: string;
  channel?: string;
}

export interface EvalAbortMessage {
  type: "evalAbort";
  runId: string;
  reason: string;
  failureStage: EvalFailureStage;
  channel?: string;
}

export interface EvalCleanupMessage {
  type: "evalCleanup";
  runId: string;
  channel?: string;
}

export interface EvalResultMessage {
  type: "evalResult";
  runId: string;
  workflowUrl: string;
  scene: string;
  task: string;
  passed: boolean;
  status: EvalStatus;
  failureStage?: EvalFailureStage;
  reason?: string;
  score?: number;
  durationMs: number;
  channel?: string;
}

export type EvalProtocolMessage =
  | RunEvalMessage
  | EvalReadyMessage
  | EvalResetMessage
  | ResetAckMessage
  | EvalStartMessage
  | EvalAbortMessage
  | EvalCleanupMessage
  | EvalResultMessage;

export function isFiniteStartPose(value: unknown): value is EvalStartPose {
  if (!value || typeof value !== "object") return false;
  const pose = value as Record<string, unknown>;
  return ["x", "y", "z", "yaw"].every(
    (key) => typeof pose[key] === "number" && Number.isFinite(pose[key]),
  );
}
