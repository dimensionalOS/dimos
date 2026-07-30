import type {
  EvalResetMessage,
  EvalStartPose,
  ResetAckMessage,
} from "../../evals/protocol.ts";
import { isFiniteStartPose } from "../../evals/protocol.ts";

export interface ResettablePhysics {
  resetPose(pose: EvalStartPose): EvalStartPose;
  clearMotion(): void;
}

/** Validate and apply an authoritative reset without depending on the WS server. */
export function applyEvalReset(
  physics: ResettablePhysics | null,
  message: EvalResetMessage,
): ResetAckMessage {
  if (!physics) {
    return {
      type: "resetAck",
      runId: message.runId,
      ok: false,
      reason: "server physics is unavailable",
    };
  }
  if (!isFiniteStartPose(message.startPose)) {
    return {
      type: "resetAck",
      runId: message.runId,
      ok: false,
      reason: "startPose requires finite x, y, z, and yaw",
    };
  }
  try {
    const pose = physics.resetPose(message.startPose);
    if (!isFiniteStartPose(pose)) {
      throw new Error("server physics returned an invalid actual pose");
    }
    return { type: "resetAck", runId: message.runId, ok: true, pose };
  } catch (error) {
    return {
      type: "resetAck",
      runId: message.runId,
      ok: false,
      reason: error instanceof Error ? error.message : String(error),
    };
  }
}
