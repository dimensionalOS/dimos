// Keyboard teleop panel. Click to focus = arm (the lease handshake runs
// through teleopMachine); armed only while the wrapper subtree has focus, so
// typing anywhere else can never drive the robot. All safety logic lives in
// the machine; this component is listeners + visuals.

import { useEffect, useState, useSyncExternalStore } from "react";
import type { FocusEvent, KeyboardEvent } from "react";
import {
  HANDLED_CODES,
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

export function TeleopPanel({ spec, teleop }: PanelProps) {
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
  return <TeleopControls spec={spec} teleop={teleop} ch={ch} />;
}

function TeleopControls({ spec, teleop, ch }: {
  spec: PanelProps["spec"];
  teleop: NonNullable<PanelProps["teleop"]>;
  ch: string;
}) {
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
    banner = <span className={styles.armed}>armed - WASD drive, QE strafe, Space stop</span>;
  } else {
    banner = (
      <span className={styles.hint}>
        click to arm{snap.reason !== null ? ` (${snap.reason})` : ""}
      </span>
    );
  }

  return (
    <PanelFrame spec={spec}>
      <div
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
        </div>
      </div>
    </PanelFrame>
  );
}
