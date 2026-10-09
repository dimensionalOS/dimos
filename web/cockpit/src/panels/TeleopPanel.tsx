// Teleop panel: two stick widgets that follow the gamepad (or the keys), the
// commanded speed, and the arm/stop state. Click to focus = arm (the lease
// handshake runs through teleopMachine); armed only while the wrapper subtree
// has focus, so typing anywhere else can never drive the robot. All safety
// logic lives in the machine; this component is listeners + visuals.

import { useEffect, useRef, useState, useSyncExternalStore } from "react";
import type { FocusEvent, KeyboardEvent, ReactNode } from "react";
import {
  HANDLED_CODES,
  STICK_AT_REST,
  stickFromGamepad,
  type StickInput,
  teleopConfigFromChannel,
  TeleopMachine,
} from "@dimos/sdk/internal/teleop";
import { useStatus } from "@dimos/sdk/react";
import { PanelFrame } from "../layout/PanelFrame.tsx";
import type { PanelProps } from "./registry.tsx";
import styles from "./TeleopPanel.module.css";

/** Standard-mapping buttons the panel acts on: A arms (by focusing the
 * pad), B is the e-stop. Sticks and the right trigger go to the machine. */
const PAD_A = 0;
const PAD_B = 1;
const PAD_POLL_HZ = 30;

/** The pad that changed most recently. Steam Input exposes several virtual
 * pads at once and only one of them carries the operator's input. */
function activeGamepad(): Gamepad | null {
  if (typeof navigator === "undefined" || typeof navigator.getGamepads !== "function") return null;
  let best: Gamepad | null = null;
  for (const pad of navigator.getGamepads()) {
    if (pad !== null && (best === null || pad.timestamp > best.timestamp)) best = pad;
  }
  return best;
}

function fmtAxis(v: number): string {
  return (v < 0 ? "−" : "+") + Math.abs(v).toFixed(2);
}

/** A stick as the operator sees it: a ring, a knob at (x, y) in -1..1, and
 * the values beneath. */
function Stick({ x, y, label, testId, data, children }: {
  x: number;
  y: number;
  label: string;
  testId: string;
  data: Record<string, string>;
  children: ReactNode;
}) {
  const cx = Math.min(Math.max(x, -1), 1);
  const cy = Math.min(Math.max(y, -1), 1);
  const live = cx !== 0 || cy !== 0;
  return (
    <div className={styles.stick} data-testid={testId} {...data}>
      <div className={styles.ring}>
        <span className={styles.cross} />
        <span
          className={live ? styles.knobLive : styles.knob}
          style={{ transform: `translate(${cx * 30}px, ${cy * 30}px)` }}
        />
      </div>
      <div className={styles.axes}>
        <span className={styles.stickLabel}>{label}</span>
        {children}
      </div>
    </div>
  );
}

export function TeleopPanel({ spec, teleop, session }: PanelProps) {
  const ch = spec.channels[0] as string | undefined;
  if (ch === undefined || teleop === undefined) {
    // No channel is a bridge authoring mistake; no teleop hooks means a
    // host (test) that cannot send - render visibly instead of crashing.
    return (
      <PanelFrame spec={spec}>
        <span className={styles.hint}>teleop panel {spec.id}: no send path bound</span>
      </PanelFrame>
    );
  }
  return <TeleopControls spec={spec} teleop={teleop} ch={ch} session={session} />;
}

/** Raw pad state for the optional joystick channel: axes to three decimals,
 * buttons as 0/1. A "standard" pad reports its triggers as analog buttons 6/7;
 * their 0..1 values go out as axes 4/5 (SDL order), since the right trigger
 * boosts continuously and 0/1 would not replay it. Equal samples publish
 * nothing. */
export function joySample(
  pad: Pick<Gamepad, "axes" | "buttons" | "mapping">,
): { axes: number[]; buttons: number[] } {
  const q = (a: number) => Math.round(a * 1000) / 1000;
  const axes = pad.axes.map(q);
  if (pad.mapping === "standard") {
    axes.push(q(pad.buttons[6]?.value ?? 0), q(pad.buttons[7]?.value ?? 0));
  }
  return { axes, buttons: pad.buttons.map((b) => (b.pressed ? 1 : 0)) };
}

function TeleopControls({ spec, teleop, ch, session }: {
  spec: PanelProps["spec"];
  teleop: NonNullable<PanelProps["teleop"]>;
  ch: string;
  session: PanelProps["session"];
}) {
  // Second channel, when the blueprint asked for it: the raw gamepad state.
  const joyCh = spec.channels[1] as string | undefined;
  const status = useStatus(teleop);
  const connected = status.transport.phase === "connected";
  // Config is read once per mount: a manifest edit changes the channel
  // params, which changes the manifest, which remounts via the App epoch.
  const [machine] = useState(() =>
    new TeleopMachine(
      teleopConfigFromChannel(status.manifest?.channels.find((c) => c.ch === ch)),
      teleop,
    )
  );
  const snap = useSyncExternalStore(machine.subscribe, machine.getSnapshot);

  useEffect(() => teleop.onMsg((msg) => machine.onRelayMsg(msg)), [teleop, machine]);
  useEffect(() => machine.connectionChanged(connected), [machine, connected]);
  useEffect(() => {
    // Missed keyups and background tabs must never keep driving.
    const onBlur = () => machine.disarm("window blurred");
    const onVisibility = () => {
      if (document.visibilityState === "hidden") machine.disarm("tab hidden");
    };
    globalThis.addEventListener("blur", onBlur);
    document.addEventListener("visibilitychange", onVisibility);
    return () => {
      globalThis.removeEventListener("blur", onBlur);
      document.removeEventListener("visibilitychange", onVisibility);
      // Unmount (robot gone, layout change): zero and release the lease.
      machine.disarm("panel unmounted");
    };
  }, [machine]);

  // Gamepad: polled, since the Gamepad API has no input events. A focuses
  // the pad (arming through the same focus path as a click), B e-stops,
  // the sticks feed the machine; everything else keeps the keyboard rules.
  const padRef = useRef<HTMLDivElement>(null);
  const [padName, setPadName] = useState<string | null>(null);
  const [padStick, setPadStick] = useState<StickInput>(STICK_AT_REST);
  useEffect(() => {
    let prevA = false;
    let prevB = false;
    let lastName: string | null = null;
    let lastStick = STICK_AT_REST;
    let lastJoy = "";
    let lastJoyAt = 0;
    const joyMinMs = 1000 / machine.config.publishHz;
    const timer = setInterval(() => {
      const pad = activeGamepad();
      const name = pad?.id ?? null;
      if (name !== lastName) {
        lastName = name;
        setPadName(name);
      }
      if (pad === null) {
        if (lastStick !== STICK_AT_REST) setPadStick(lastStick = STICK_AT_REST);
        return;
      }
      const a = pad.buttons[PAD_A]?.pressed ?? false;
      const b = pad.buttons[PAD_B]?.pressed ?? false;
      if (a && !prevA) padRef.current?.focus();
      if (b && !prevB) machine.estop();
      prevA = a;
      prevB = b;
      const stick = stickFromGamepad(pad.axes, pad.buttons, pad.mapping);
      machine.stick(stick);
      if (
        stick.vx !== lastStick.vx || stick.vy !== lastStick.vy || stick.wz !== lastStick.wz ||
        stick.boost !== lastStick.boost
      ) {
        setPadStick(lastStick = stick);
      }
      if (joyCh !== undefined && session !== undefined) {
        const sample = joySample(pad);
        const key = JSON.stringify(sample);
        const now = performance.now();
        if (key !== lastJoy && now - lastJoyAt >= joyMinMs) {
          lastJoy = key;
          lastJoyAt = now;
          // Fire-and-forget: a dropped sample is replaced by the next change.
          session.publish(joyCh, sample).catch(() => {});
        }
      }
    }, 1000 / PAD_POLL_HZ);
    return () => clearInterval(timer);
  }, [machine, joyCh, session]);

  // Arm from focus AND click: after teleop_held or a reconnect the pad can
  // still hold focus, and clicking an already-focused element fires no focus
  // event. arm() no-ops unless disarmed, so the pair never double-sends.
  const arm = () => {
    if (connected) machine.arm();
  };
  const onBlur = (e: FocusEvent<HTMLDivElement>) => {
    // Focus moves within the panel (relatedTarget still inside) do not
    // disarm; leaving the subtree does.
    if (e.relatedTarget instanceof Node && e.currentTarget.contains(e.relatedTarget)) return;
    machine.disarm("focus lost");
  };
  const onKeyDown = (e: KeyboardEvent<HTMLDivElement>) => {
    if (!HANDLED_CODES.has(e.code)) return;
    e.preventDefault();
    if (e.repeat) return;
    if (e.code === "Escape") {
      // Blur (which disarms) so the next click re-focuses and re-arms.
      e.currentTarget.blur();
    } else if (e.code === "Space") {
      machine.estop();
    } else {
      machine.keyDown(e.code);
    }
  };
  const onKeyUp = (e: KeyboardEvent<HTMLDivElement>) => {
    if (!HANDLED_CODES.has(e.code)) return;
    e.preventDefault();
    machine.keyUp(e.code);
  };

  // What the sticks show: the pad when one is present, else the keys as a
  // deflection (W = full forward).
  const { maxLinear, maxAngular, boost } = machine.config;
  const lin = maxLinear * (snap.boosted ? boost : 1);
  const ang = maxAngular * (snap.boosted ? boost : 1);
  const shown: StickInput = padName !== null
    ? padStick
    : { vx: snap.vx / lin, vy: snap.vy / lin, wz: snap.wz / ang, boost: 0 };

  const state = !connected ? "stopped" : snap.phase;
  let banner;
  if (state === "stopped") {
    banner = <span className={styles.stopped}>connection lost</span>;
  } else if (state === "arming") {
    banner = <span className={styles.hint}>requesting teleop...</span>;
  } else if (state === "armed") {
    banner = (
      <span className={styles.armed}>
        {padName === null ? "armed · WASD drive · QE strafe · Space stop" : "armed · B stops"}
      </span>
    );
  } else {
    banner = (
      <span className={styles.hint}>
        {padName === null ? "click to arm" : "click or press A to arm"}
        {snap.reason !== null ? ` (${snap.reason})` : ""}
      </span>
    );
  }

  return (
    <PanelFrame spec={spec}>
      <div
        ref={padRef}
        tabIndex={0}
        role="application"
        aria-label={`keyboard teleop ${ch}`}
        className={state === "armed" ? styles.padArmed : styles.pad}
        data-testid={`teleop-${ch}`}
        data-state={state}
        onFocus={arm}
        onClick={arm}
        onBlur={onBlur}
        onKeyDown={onKeyDown}
        onKeyUp={onKeyUp}
      >
        <div className={styles.sticks}>
          <div className={styles.status}>
            {banner}
            {padName !== null && (
              <span className={styles.padName} data-testid={`teleop-${ch}-pad`}>{padName}</span>
            )}
          </div>
          <Stick
            x={0 - shown.vy}
            y={0 - shown.vx}
            label="move"
            testId={`teleop-${ch}-stick-left`}
            data={{ "data-vx": shown.vx.toFixed(2), "data-vy": shown.vy.toFixed(2) }}
          >
            <span>x {fmtAxis(shown.vx)}</span>
            <span>y {fmtAxis(shown.vy)}</span>
          </Stick>
          <div className={styles.speed} data-testid={`teleop-${ch}-readout`}>
            <span className={styles.speedValue}>
              {Math.hypot(snap.vx, snap.vy).toFixed(2)}
              <small>m/s</small>
            </span>
            <span className={styles.speedValue}>
              {snap.wz.toFixed(2)}
              <small>rad/s</small>
            </span>
            <span className={snap.boosted ? styles.boost : styles.hint}>
              {snap.boosted ? "boost" : `max ${machine.config.maxLinear.toFixed(1)} m/s`}
            </span>
          </div>
          <Stick
            x={0 - shown.wz}
            y={0}
            label="turn"
            testId={`teleop-${ch}-stick-right`}
            data={{ "data-wz": shown.wz.toFixed(2) }}
          >
            <span>yaw {fmtAxis(shown.wz)}</span>
          </Stick>
        </div>
      </div>
    </PanelFrame>
  );
}
