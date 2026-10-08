// Keyboard teleop panel. Click to focus = arm (the lease handshake runs
// through teleopMachine); armed only while the wrapper subtree has focus, so
// typing anywhere else can never drive the robot. All safety logic lives in
// the machine; this component is listeners + visuals.

import { useEffect, useRef, useState, useSyncExternalStore } from "react";
import type { FocusEvent, KeyboardEvent } from "react";
import {
  HANDLED_CODES,
  stickFromGamepad,
  teleopConfigFromChannel,
  TeleopMachine,
  type TeleopSnapshot,
} from "@dimos/sdk/internal/teleop";
import { useStatus } from "@dimos/sdk/react";
import { PanelFrame } from "../layout/PanelFrame.tsx";
import type { PanelProps } from "./registry.tsx";
import styles from "./TeleopPanel.module.css";

const KEY_ROWS: { code: string; label: string }[][] = [
  [
    { code: "KeyQ", label: "Q" },
    { code: "KeyW", label: "W" },
    { code: "KeyE", label: "E" },
  ],
  [
    { code: "KeyA", label: "A" },
    { code: "KeyS", label: "S" },
    { code: "KeyD", label: "D" },
  ],
];

function pressedClass(snap: TeleopSnapshot, code: string): string {
  return snap.pressed.has(code) ? styles.keyDown : styles.key;
}

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
 * buttons as 0/1. Equal samples publish nothing. */
function joySample(pad: Gamepad): { axes: number[]; buttons: number[] } {
  return {
    axes: pad.axes.map((a) => Math.round(a * 1000) / 1000),
    buttons: pad.buttons.map((b) => (b.pressed ? 1 : 0)),
  };
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
  useEffect(() => {
    let prevA = false;
    let prevB = false;
    let lastName: string | null = null;
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
      if (pad === null) return;
      const a = pad.buttons[PAD_A]?.pressed ?? false;
      const b = pad.buttons[PAD_B]?.pressed ?? false;
      if (a && !prevA) padRef.current?.focus();
      if (b && !prevB) machine.estop();
      prevA = a;
      prevB = b;
      machine.stick(stickFromGamepad(pad.axes, pad.buttons, pad.mapping));
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

  const state = !connected ? "stopped" : snap.phase;
  let banner;
  if (state === "stopped") {
    banner = <span className={styles.stopped}>connection lost</span>;
  } else if (state === "arming") {
    banner = <span className={styles.hint}>requesting teleop...</span>;
  } else if (state === "armed") {
    banner = (
      <span className={styles.armed}>
        {padName === null
          ? "armed - WASD drive, QE strafe, Space stop"
          : "armed - sticks drive, RT boost, B stop"}
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
        <div className={styles.banner}>{banner}</div>
        <div className={styles.cluster}>
          {KEY_ROWS.map((row) => (
            <div key={row[0].code} className={styles.keyRow}>
              {row.map(({ code, label }) => (
                <span
                  key={code}
                  className={pressedClass(snap, code)}
                  data-testid={`teleop-key-${label}`}
                  data-pressed={snap.pressed.has(code) || undefined}
                >
                  {label}
                </span>
              ))}
            </div>
          ))}
        </div>
        <div className={styles.readout} data-testid={`teleop-${ch}-readout`}>
          <span>vx {snap.vx.toFixed(2)}</span>
          <span>vy {snap.vy.toFixed(2)}</span>
          <span>wz {snap.wz.toFixed(2)}</span>
          {snap.boosted && <span className={styles.boost}>boost</span>}
          {padName !== null && <span data-testid={`teleop-${ch}-pad`}>pad: {padName}</span>}
        </div>
      </div>
    </PanelFrame>
  );
}
