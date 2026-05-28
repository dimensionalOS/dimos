export type Mode = "voice" | "manual";

/** Documented Unitree sport commands (WEBAPP-BRIEF §5). */
export type UnitreeCommand =
  | "StandUp"
  | "StandDown"
  | "Sit"
  | "Hello"
  | "Stretch"
  | "Dance1"
  | "Dance2";

/** A quick-action button. `command` is the natural-language query sent to the
 *  agent via /submit_query (e.g. "sit", "stand up") — NOT the direct sport command. */
export interface QuickAction {
  label: string;
  command: string;
}

/** Continuous drive vector from the joystick, components roughly in [-1, 1]. */
export interface MoveCommand {
  /** forward (+) / backward (−) */
  vx: number;
  /** strafe — reserved, unused for now */
  vy: number;
  /** yaw: turn left (−) / right (+) */
  turn: number;
}

export type AgentPhase = "idle" | "running" | "error";

/** Snapshot pushed over the `agent_state` SSE stream. */
export interface AgentState {
  phase: AgentPhase;
  task?: string;
  skill?: string;
  /** 0..1 */
  progress?: number;
  message?: string;
  /** set when a stream frame couldn't be parsed as JSON */
  raw?: string;
}
