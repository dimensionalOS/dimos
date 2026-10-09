// Teleop state machine: pure TS, no React, no DOM listeners. The panel feeds
// it key/focus events; it owns the lease handshake, the publish cadence, and
// the zeroing rules, and reports state through a tiny external-store surface.
//
// Key semantics copied from the pygame module
// (dimos/robot/unitree/keyboard_teleop.py): W/S = +-linear.x, Q/E =
// +-linear.y (strafe), A/D = +-angular.z, assignment not accumulation (S
// wins over W, E over Q, D over A), Shift held = boost, Space = e-stop that
// clears held keys but keeps running (here: stays armed). No slow modifier
// (Ctrl+W/Ctrl+Q are browser-reserved and would close the tab mid-drive).
//
// Safety shape (hop 1 of the chain; the relay lease and the bridge deadman
// are hops 2 and 3): twists repeat at publishHz while a motion key is held;
// the last release sends a zero immediately plus two repeats, then goes
// silent; every disarm sends one zero plus teleop_stop (the relay and
// bridge cover the lossy rest); Space bursts stop datagrams and puts one
// Stop on the ordered control stream.

import type { Msg } from "@dimos/shared";
import type { ChannelSpec } from "@dimos/shared/manifest";
import type { StatusStore } from "../store.ts";

/** The session's teleop surface handed to panels (registry PanelProps). */
export interface TeleopHooks {
  /** Send on the ordered control stream (teleop_start/teleop_stop, Stop). */
  control(msg: Msg): void;
  /** Fire-and-forget datagram (twist/stop at publish rate). */
  datagram(msg: Msg): void;
  /** Teleop-routed relay replies: teleop_started and the teleop_held error. */
  onMsg(cb: (msg: Msg) => void): () => void;
  status: StatusStore;
}

// Internal until W9: the session registers its raw send hooks here so the
// cockpit can reach them without them living on the public read-only
// Session. W1 consumers get no teleop surface; the safe public facade
// arrives with W9.
const hooksBySession = new WeakMap<object, TeleopHooks>();

export function registerTeleopHooks(session: object, hooks: TeleopHooks): void {
  hooksBySession.set(session, hooks);
}

export function teleopHooks(session: object): TeleopHooks {
  const hooks = hooksBySession.get(session);
  if (hooks === undefined) throw new Error("session has no registered teleop hooks");
  return hooks;
}

export interface TeleopConfig {
  maxLinear: number;
  maxAngular: number;
  boost: number;
  publishHz: number;
}

export const TELEOP_DEFAULTS: TeleopConfig = {
  maxLinear: 0.8,
  maxAngular: 1.0,
  boost: 2.0,
  publishHz: 15,
};

function finitePositive(value: unknown, fallback: number): number {
  return typeof value === "number" && Number.isFinite(value) && value > 0 ? value : fallback;
}

/** Config from the manifest tx channel: params for speeds, maxHz for the
 * cadence. Params crossed the wire, so junk falls back per-field. */
export function teleopConfigFromChannel(spec: ChannelSpec | undefined): TeleopConfig {
  const params = spec?.params ?? {};
  return {
    maxLinear: finitePositive(params.maxLinear, TELEOP_DEFAULTS.maxLinear),
    maxAngular: finitePositive(params.maxAngular, TELEOP_DEFAULTS.maxAngular),
    boost: finitePositive(params.boost, TELEOP_DEFAULTS.boost),
    publishHz: finitePositive(spec?.maxHz, TELEOP_DEFAULTS.publishHz),
  };
}

const MOTION_CODES = new Set(["KeyW", "KeyA", "KeyS", "KeyD", "KeyQ", "KeyE"]);
const BOOST_CODES = new Set(["ShiftLeft", "ShiftRight"]);
/** Codes the panel intercepts (preventDefault) while it has focus. */
export const HANDLED_CODES = new Set([...MOTION_CODES, ...BOOST_CODES, "Space", "Escape"]);

/** Analog input from a gamepad, each axis -1..1 after the deadzone, boost 0..1
 * (the right trigger). Zero on every field means "stick at rest". */
export interface StickInput {
  vx: number;
  vy: number;
  wz: number;
  boost: number;
}

export const STICK_AT_REST: StickInput = { vx: 0, vy: 0, wz: 0, boost: 0 };

/** Dead zone of the browser's standard gamepad mapping, rescaled so the
 * edge of the zone reads 0 and full deflection 1. */
export const STICK_DEADZONE = 0.15;

function deadzone(value: number): number {
  const magnitude = Math.abs(value);
  if (magnitude < STICK_DEADZONE) return 0;
  return Math.sign(value) * (magnitude - STICK_DEADZONE) / (1 - STICK_DEADZONE);
}

/** Gamepad sample to a StickInput: left stick drives (up = forward, left =
 * +y), right stick X yaws (left = +wz), right trigger boosts. The browser's
 * "standard" mapping puts the right stick on axes 2/3 and the trigger on
 * button 7. A pad the browser does not recognise (a handheld's built-in
 * xpad sticks, say) comes raw in evdev order: right stick X on axis 3 and
 * the trigger on axis 5 running -1..1. */
export function stickFromGamepad(
  axes: readonly number[],
  buttons: readonly { value: number }[],
  mapping = "standard",
): StickInput {
  const standard = mapping === "standard";
  const boost = standard ? buttons[7]?.value ?? 0 : ((axes[5] ?? -1) + 1) / 2;
  return {
    vx: 0 - deadzone(axes[1] ?? 0),
    vy: 0 - deadzone(axes[0] ?? 0),
    wz: 0 - deadzone(axes[standard ? 2 : 3] ?? 0),
    boost: Math.min(1, Math.max(0, boost)),
  };
}

export function stickActive(stick: StickInput): boolean {
  return stick.vx !== 0 || stick.vy !== 0 || stick.wz !== 0;
}

/** Release/e-stop burst schedule: send now, repeat twice over 200 ms. */
const BURST_DELAYS_MS = [100, 200];

export type TeleopPhase = "disarmed" | "arming" | "armed";

export interface TeleopSnapshot {
  phase: TeleopPhase;
  /** Why the machine is disarmed (shown by the panel), or null. */
  reason: string | null;
  pressed: ReadonlySet<string>;
  vx: number;
  vy: number;
  wz: number;
  boosted: boolean;
}

export class TeleopMachine {
  readonly config: TeleopConfig;
  #send: Pick<TeleopHooks, "control" | "datagram">;
  #phase: TeleopPhase = "disarmed";
  #reason: string | null = null;
  #pressed = new Set<string>();
  #stick: StickInput = STICK_AT_REST;
  #seq = 0;
  #command = { vx: 0, vy: 0, wz: 0, boosted: false };
  #interval: ReturnType<typeof setInterval> | null = null;
  #burstTimers: ReturnType<typeof setTimeout>[] = [];
  #listeners = new Set<() => void>();
  #snapshot: TeleopSnapshot;

  constructor(config: TeleopConfig, send: Pick<TeleopHooks, "control" | "datagram">) {
    this.config = config;
    this.#send = send;
    this.#snapshot = this.#buildSnapshot();
  }

  subscribe = (cb: () => void): () => void => {
    this.#listeners.add(cb);
    return () => this.#listeners.delete(cb);
  };

  getSnapshot = (): TeleopSnapshot => this.#snapshot;

  /** Request the lease; ARMED only once the relay acks with teleop_started. */
  arm(): void {
    if (this.#phase !== "disarmed") return;
    this.#phase = "arming";
    this.#reason = null;
    this.#send.control({ t: "teleop_start" });
    this.#emit();
  }

  onRelayMsg(msg: Msg): void {
    if (msg.t === "teleop_started") {
      if (this.#phase === "arming") {
        this.#phase = "armed";
        this.#emit();
      }
    } else if (msg.t === "error" && msg.code === "teleop_held") {
      // Refused: another viewer holds the lease. Local-only teardown (there
      // is nothing of ours to zero or release).
      this.#stopTimers();
      this.#pressed.clear();
      this.#stick = STICK_AT_REST;
      this.#phase = "disarmed";
      this.#reason = "teleop held by another viewer";
      this.#emit();
    }
  }

  /** Zero, release the lease, and go idle. Every disarm trigger (Esc, focus
   * loss, blur, hidden tab, unmount) funnels here. */
  disarm(reason: string): void {
    if (this.#phase === "disarmed") return;
    const wasArmed = this.#phase === "armed";
    this.#stopTimers();
    this.#pressed.clear();
    this.#stick = STICK_AT_REST;
    this.#phase = "disarmed";
    this.#reason = reason;
    if (wasArmed) {
      // One immediate zero; the repeats would race the lease release below
      // and be gated at the relay. The relay's robot-ward teleop_stop and
      // the bridge deadman cover a lost datagram.
      this.#sendTwist(0, 0, 0);
    }
    this.#send.control({ t: "teleop_stop" });
    this.#emit();
  }

  /** Transport phase edges; disconnect disarms without wire sends. */
  connectionChanged(connected: boolean): void {
    if (connected || this.#phase === "disarmed") return;
    this.#stopTimers();
    this.#pressed.clear();
    this.#stick = STICK_AT_REST;
    this.#phase = "disarmed";
    this.#reason = "connection lost";
    this.#emit();
  }

  /** Space: clear all held keys and burst stops. Stays armed, like the
   * pygame module keeps running after its e-stop. */
  estop(): void {
    if (this.#phase !== "armed") return;
    this.#stopTimers();
    this.#pressed.clear();
    this.#stick = STICK_AT_REST;
    this.#send.control({ t: "stop", seq: ++this.#seq, ts: Date.now() / 1000 });
    this.#sendStop();
    this.#burstTimers = BURST_DELAYS_MS.map((ms) => setTimeout(() => this.#sendStop(), ms));
    this.#emit();
  }

  keyDown(code: string): void {
    if (!MOTION_CODES.has(code) && !BOOST_CODES.has(code)) return;
    if (this.#phase !== "armed") return;
    this.#pressed.add(code);
    if (this.#anyMotionHeld()) {
      this.#cancelBurst();
      this.#sendCurrent();
      this.#interval ??= setInterval(() => this.#sendCurrent(), 1000 / this.config.publishHz);
    }
    this.#emit();
  }

  keyUp(code: string): void {
    if (!this.#pressed.delete(code)) return;
    if (this.#phase === "armed" && this.#anyMotionHeld()) {
      this.#sendCurrent();
    } else if (this.#phase === "armed" && this.#interval !== null) {
      // Last motion key released: zero now plus two repeats, then silence.
      this.#stopInterval();
      this.#sendTwist(0, 0, 0);
      this.#burstTimers = BURST_DELAYS_MS.map((ms) =>
        setTimeout(() => this.#sendTwist(0, 0, 0), ms)
      );
    }
    this.#emit();
  }

  /** Gamepad sample. Same cadence rules as the keys: motion starts the
   * publish interval, the return to rest sends the zero burst. Samples that
   * change nothing are free, so the panel can poll at any rate. */
  stick(input: StickInput): void {
    if (this.#phase !== "armed") return;
    const prev = this.#stick;
    if (
      prev.vx === input.vx && prev.vy === input.vy && prev.wz === input.wz &&
      prev.boost === input.boost
    ) return;
    this.#stick = input;
    if (this.#anyMotionHeld()) {
      this.#cancelBurst();
      this.#sendCurrent();
      this.#interval ??= setInterval(() => this.#sendCurrent(), 1000 / this.config.publishHz);
    } else if (this.#interval !== null) {
      this.#stopInterval();
      this.#sendTwist(0, 0, 0);
      this.#burstTimers = BURST_DELAYS_MS.map((ms) =>
        setTimeout(() => this.#sendTwist(0, 0, 0), ms)
      );
    }
    this.#emit();
  }

  #anyMotionHeld(): boolean {
    if (stickActive(this.#stick)) return true;
    for (const code of this.#pressed) if (MOTION_CODES.has(code)) return true;
    return false;
  }

  #sendCurrent(): void {
    const held = this.#pressed;
    let vx = 0;
    let vy = 0;
    let wz = 0;
    if (held.has("KeyW")) vx = this.config.maxLinear;
    if (held.has("KeyS")) vx = -this.config.maxLinear;
    if (held.has("KeyQ")) vy = this.config.maxLinear;
    if (held.has("KeyE")) vy = -this.config.maxLinear;
    if (held.has("KeyA")) wz = this.config.maxAngular;
    if (held.has("KeyD")) wz = -this.config.maxAngular;
    const stick = this.#stick;
    if (stick.vx !== 0) vx = stick.vx * this.config.maxLinear;
    if (stick.vy !== 0) vy = stick.vy * this.config.maxLinear;
    if (stick.wz !== 0) wz = stick.wz * this.config.maxAngular;
    const shift = held.has("ShiftLeft") || held.has("ShiftRight");
    const k = shift ? this.config.boost : 1 + (this.config.boost - 1) * stick.boost;
    this.#sendTwist(vx * k, vy * k, wz * k, shift || stick.boost > 0.5);
  }

  #sendTwist(vx: number, vy: number, wz: number, boosted = false): void {
    this.#command = { vx, vy, wz, boosted };
    this.#send.datagram({ t: "twist", vx, vy, wz, seq: ++this.#seq, ts: Date.now() / 1000 });
    this.#emit();
  }

  #sendStop(): void {
    this.#command = { vx: 0, vy: 0, wz: 0, boosted: false };
    this.#send.datagram({ t: "stop", seq: ++this.#seq, ts: Date.now() / 1000 });
    this.#emit();
  }

  #stopInterval(): void {
    if (this.#interval !== null) {
      clearInterval(this.#interval);
      this.#interval = null;
    }
  }

  #cancelBurst(): void {
    for (const timer of this.#burstTimers) clearTimeout(timer);
    this.#burstTimers = [];
  }

  #stopTimers(): void {
    this.#stopInterval();
    this.#cancelBurst();
  }

  #buildSnapshot(): TeleopSnapshot {
    return {
      phase: this.#phase,
      reason: this.#reason,
      pressed: new Set(this.#pressed),
      ...this.#command,
    };
  }

  #emit(): void {
    this.#snapshot = this.#buildSnapshot();
    for (const cb of this.#listeners) cb();
  }
}
