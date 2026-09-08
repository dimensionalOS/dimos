import { describe, expect, it } from "vitest";
import {
  derivePolicyUi,
  livePending,
  navCancellable,
  PENDING_MS,
  type PolicyEntry,
  type PolicyState,
  readNavState,
  readPolicyState,
} from "./controlPolicy.ts";

const NAMES: [string, PolicyEntry["kind"]][] = [
  ["walk", "base"],
  ["stand", "base"],
  ["roller", "base"],
  ["roller_crouch", "base"],
  ["sitstand", "posture"],
  ["kick_left", "oneshot"],
  ["kick_right", "oneshot"],
  ["roulade", "oneshot"],
  ["ground_pick", "oneshot"],
];

function state(partial: Partial<PolicyState> = {}): PolicyState {
  return {
    variant: "default",
    active: "walk",
    base: "walk",
    seated: false,
    fallen: false,
    locked: false,
    oneshot: null,
    policies: NAMES.map(([name, kind]) => ({ name, kind, available: true, reason: null })),
    last_error: null,
    t: 0,
    ...partial,
  };
}

const byName = (buttons: ReturnType<typeof derivePolicyUi>) =>
  Object.fromEntries(buttons.map((b) => [b.name, b.state]));

describe("derivePolicyUi", () => {
  it("returns nothing without state and keeps the bridge's display order", () => {
    expect(derivePolicyUi(null, [], 0)).toEqual([]);
    const buttons = derivePolicyUi(state(), [], 0);
    expect(buttons.map((b) => b.name)).toEqual(NAMES.map(([n]) => n));
    expect(buttons.map((b) => b.kind)).toEqual(NAMES.map(([, k]) => k));
  });

  it("marks the selected base gait active and the rest idle; base starts, others toggle", () => {
    const buttons = derivePolicyUi(state(), [], 0);
    expect(byName(buttons)).toMatchObject({ walk: "active", stand: "idle", sitstand: "idle" });
    expect(buttons.find((b) => b.name === "walk")?.action).toBe("start");
    expect(buttons.find((b) => b.name === "sitstand")?.action).toBe("toggle");
    expect(buttons.find((b) => b.name === "roulade")?.action).toBe("toggle");
    expect(buttons.find((b) => b.name === "roller_crouch")?.label).toBe("roller crouch");
    expect(buttons.find((b) => b.name === "sitstand")?.label).toBe("sit / stand");
  });

  it("shows a running oneshot with progress and the posture policy active while seated", () => {
    const running = derivePolicyUi(
      state({ active: "roulade", oneshot: { name: "roulade", progress: 0.4 } }),
      [],
      0,
    );
    const roulade = running.find((b) => b.name === "roulade");
    expect(roulade?.state).toBe("running");
    expect(roulade?.progress).toBe(0.4);
    expect(byName(running).walk).toBe("active"); // still the selected base

    const seated = derivePolicyUi(state({ seated: true, active: "stand", base: "stand" }), [], 0);
    expect(byName(seated)).toMatchObject({ sitstand: "active", stand: "active", walk: "idle" });
    const transitioning = derivePolicyUi(state({ active: "sitstand" }), [], 0);
    expect(byName(transitioning).sitstand).toBe("running");
  });

  it("surfaces unavailable policies with their reason, above pending", () => {
    const s = state();
    s.policies[0] = { name: "walk", kind: "base", available: false, reason: "seated" };
    s.base = "stand";
    s.active = "stand";
    const buttons = derivePolicyUi(s, [{ policy: "walk", action: "start", at: 0 }], 100);
    const walk = buttons.find((b) => b.name === "walk");
    expect(walk?.state).toBe("unavailable");
    expect(walk?.reason).toBe("seated");
    expect(buttons.find((b) => b.name === "stand")?.reason).toBeNull();
  });

  it("shows a click as pending for PENDING_MS, then falls back to the state", () => {
    const pending = [{ policy: "kick_left", action: "toggle" as const, at: 1000 }];
    expect(byName(derivePolicyUi(state(), pending, 1000 + PENDING_MS - 1)).kick_left).toBe(
      "pending",
    );
    expect(byName(derivePolicyUi(state(), pending, 1000 + PENDING_MS)).kick_left).toBe("idle");
    expect(livePending(pending, 1000 + PENDING_MS - 1)).toEqual(pending);
    expect(livePending(pending, 1000 + PENDING_MS)).toEqual([]);
    // Once the state reflects the request the button is running, not pending.
    const caughtUp = state({ active: "kick_left", oneshot: { name: "kick_left", progress: 0 } });
    expect(byName(derivePolicyUi(caughtUp, pending, 1100)).kick_left).toBe("running");
  });

  it("disables everything but the running/active/pending buttons while locked", () => {
    const locked = state({ locked: true, active: "braking", base: "walk" });
    const buttons = derivePolicyUi(
      locked,
      [{ policy: "stand", action: "start", at: 0 }],
      100,
    );
    expect(byName(buttons)).toMatchObject({
      walk: "active",
      stand: "pending",
      roller: "disabled",
      kick_left: "disabled",
    });
  });
});

describe("readPolicyState / readNavState", () => {
  it("accepts the policy.json.v1 shape and skips malformed entries", () => {
    const s = readPolicyState({
      variant: "default",
      active: "braking",
      base: "walk",
      seated: false,
      fallen: true,
      locked: true,
      oneshot: { name: "kick_left", progress: 0.25 },
      policies: [
        { name: "walk", kind: "base", available: true, reason: null },
        { name: "bad", kind: "weird" },
        "junk",
        { name: "kick_left", kind: "oneshot", available: false, reason: "fallen" },
      ],
      last_error: "policy refused",
      t: 12.5,
    });
    expect(s).toEqual({
      variant: "default",
      active: "braking",
      base: "walk",
      seated: false,
      fallen: true,
      locked: true,
      oneshot: { name: "kick_left", progress: 0.25 },
      policies: [
        { name: "walk", kind: "base", available: true, reason: null },
        { name: "kick_left", kind: "oneshot", available: false, reason: "fallen" },
      ],
      last_error: "policy refused",
      t: 12.5,
    });
    expect(readPolicyState({ variant: "x" })).toBeNull();
    expect(readPolicyState(null)).toBeNull();
  });

  it("reads navstate.json.v1 and knows which phases can be cancelled", () => {
    const following = readNavState({
      state: "following_path",
      goal: { x: 1.2, y: -0.5, yaw: 3.1 },
      since: 5,
      t: 6,
    });
    expect(following).toEqual({
      state: "following_path",
      goal: { x: 1.2, y: -0.5, yaw: 3.1 },
      since: 5,
      t: 6,
    });
    expect(navCancellable(following)).toBe(true);
    expect(navCancellable(readNavState({ state: "recovery", goal: null }))).toBe(true);
    expect(navCancellable(readNavState({ state: "reached", goal: { x: 0, y: 0 } }))).toBe(false);
    expect(navCancellable(null)).toBe(false);
    expect(readNavState({ state: "flying" })).toBeNull();
    expect(readNavState({ state: "idle", goal: { x: "1" } })?.goal).toBeNull();
  });
});
