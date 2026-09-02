// Pure state for the control strip: policy.json.v1 + the viewer's own recent
// clicks -> one display state per policy button. No React, no clock of its
// own (the caller passes `now`), so the optimistic-pending rule is testable
// without timers.

import { readNumber, readString } from "./panelParams.ts";

export type PolicyKind = "base" | "posture" | "oneshot";
export type PolicyAction = "start" | "stop" | "toggle";

export interface PolicyEntry {
  name: string;
  kind: PolicyKind;
  available: boolean;
  reason: string | null;
}

/** One policy.json.v1 frame, validated (see readPolicyState). */
export interface PolicyState {
  variant: string;
  /** Running policy name, or "braking" / "standing_up" between policies. */
  active: string | null;
  /** The selected base gait (walk / stand / roller). */
  base: string | null;
  seated: boolean;
  fallen: boolean;
  /** A transition is in flight: requests are refused until it settles. */
  locked: boolean;
  oneshot: { name: string; progress: number } | null;
  policies: PolicyEntry[];
  last_error: string | null;
  t: number;
}

export interface PendingRequest {
  policy: string;
  action: PolicyAction;
  /** Browser ms of the click. */
  at: number;
}

/** How long a click shows as pending before the state must have caught up. */
export const PENDING_MS = 1500;

export type PolicyButtonState =
  | "idle"
  | "active"
  | "running"
  | "pending"
  | "unavailable"
  | "disabled";

export interface PolicyButton {
  name: string;
  kind: PolicyKind;
  label: string;
  state: PolicyButtonState;
  /** 0..1 while a oneshot is running, else null. */
  progress: number | null;
  /** Why the policy is unavailable (state "unavailable"), else null. */
  reason: string | null;
  /** The command a click sends. */
  action: PolicyAction;
}

const LABELS: Record<string, string> = {
  roller_crouch: "roller crouch",
  sitstand: "sit / stand",
  kick_left: "kick left",
  kick_right: "kick right",
  ground_pick: "ground pick",
};

export function policyLabel(name: string): string {
  return LABELS[name] ?? name.replace(/_/g, " ");
}

function readKind(v: unknown): PolicyKind | null {
  return v === "base" || v === "posture" || v === "oneshot" ? v : null;
}

export function readPolicyState(v: unknown): PolicyState | null {
  if (typeof v !== "object" || v === null) return null;
  const o = v as Record<string, unknown>;
  if (!Array.isArray(o.policies)) return null;
  const policies: PolicyEntry[] = [];
  for (const p of o.policies) {
    if (typeof p !== "object" || p === null) continue;
    const e = p as Record<string, unknown>;
    const name = readString(e.name);
    const kind = readKind(e.kind);
    if (name === null || kind === null) continue;
    policies.push({
      name,
      kind,
      available: e.available !== false,
      reason: readString(e.reason),
    });
  }
  let oneshot: PolicyState["oneshot"] = null;
  if (typeof o.oneshot === "object" && o.oneshot !== null) {
    const os = o.oneshot as Record<string, unknown>;
    const name = readString(os.name);
    if (name !== null) oneshot = { name, progress: readNumber(os.progress) ?? 0 };
  }
  return {
    variant: readString(o.variant) ?? "",
    active: readString(o.active),
    base: readString(o.base),
    seated: o.seated === true,
    fallen: o.fallen === true,
    locked: o.locked === true,
    oneshot,
    policies,
    last_error: readString(o.last_error),
    t: readNumber(o.t) ?? 0,
  };
}

/** Keep the requests that can still show as pending. */
export function livePending(pending: readonly PendingRequest[], now: number): PendingRequest[] {
  return pending.filter((p) => now - p.at < PENDING_MS);
}

/**
 * Per-button display state. Precedence, highest first: running (the state
 * says this policy is executing) > active (selected base gait, or seated for
 * the posture policy) > unavailable (the state refuses it) > pending (a click
 * within PENDING_MS the state has not reflected yet) > disabled (a transition
 * is in flight) > idle. Base policies are started, the rest toggled.
 */
export function derivePolicyUi(
  state: PolicyState | null,
  pending: readonly PendingRequest[],
  now: number,
): PolicyButton[] {
  if (state === null) return [];
  const live = livePending(pending, now);
  return state.policies.map((p) => {
    const running = (state.oneshot !== null && state.oneshot.name === p.name) ||
      (state.active === p.name && p.kind !== "base");
    const active = p.kind === "base"
      ? state.base === p.name
      : p.kind === "posture"
      ? state.seated
      : false;
    const isPending = live.some((r) => r.policy === p.name);
    let buttonState: PolicyButtonState;
    if (running) buttonState = "running";
    else if (active) buttonState = "active";
    else if (!p.available) buttonState = "unavailable";
    else if (isPending) buttonState = "pending";
    else if (state.locked) buttonState = "disabled";
    else buttonState = "idle";
    return {
      name: p.name,
      kind: p.kind,
      label: policyLabel(p.name),
      state: buttonState,
      progress: state.oneshot !== null && state.oneshot.name === p.name
        ? Math.min(1, Math.max(0, state.oneshot.progress))
        : null,
      reason: buttonState === "unavailable" ? p.reason : null,
      action: p.kind === "base" ? "start" : "toggle",
    };
  });
}

export const GROUPS: { kind: PolicyKind; title: string }[] = [
  { kind: "base", title: "Base" },
  { kind: "posture", title: "Posture" },
  { kind: "oneshot", title: "Actions" },
];

export type NavPhase =
  | "idle"
  | "following_path"
  | "recovery"
  | "reached"
  | "cancelled"
  | "no_path";

export interface NavState {
  state: NavPhase;
  goal: { x: number; y: number; yaw: number } | null;
  since: number;
  t: number;
}

const NAV_PHASES = new Set<string>([
  "idle",
  "following_path",
  "recovery",
  "reached",
  "cancelled",
  "no_path",
]);

export function readNavState(v: unknown): NavState | null {
  if (typeof v !== "object" || v === null) return null;
  const o = v as Record<string, unknown>;
  const state = readString(o.state);
  if (state === null || !NAV_PHASES.has(state)) return null;
  let goal: NavState["goal"] = null;
  if (typeof o.goal === "object" && o.goal !== null) {
    const g = o.goal as Record<string, unknown>;
    const x = readNumber(g.x);
    const y = readNumber(g.y);
    if (x !== null && y !== null) goal = { x, y, yaw: readNumber(g.yaw) ?? 0 };
  }
  return {
    state: state as NavPhase,
    goal,
    since: readNumber(o.since) ?? 0,
    t: readNumber(o.t) ?? 0,
  };
}

/** A goal is in progress and worth a cancel button. */
export function navCancellable(nav: NavState | null): boolean {
  return nav !== null && (nav.state === "following_path" || nav.state === "recovery");
}
