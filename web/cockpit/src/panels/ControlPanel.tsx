// Control strip: mode switch (Teleop | Agent), the policy buttons grouped
// Base / Posture / Actions, and status chips (variant, fallen, locked, last
// error). Navigation state belongs to the map panel, next to the goal marker
// it describes. Every command is one ui_command tx; the
// clicked button shows as pending for PENDING_MS unless the policy state
// catches up first (derivePolicyUi). No keyboard shortcuts: WASD belongs to
// the teleop pad, and every button preventDefaults mousedown so a click here
// never takes focus (and so the lease) away from it.

import { type MouseEvent, useEffect, useState } from "react";
import { PanelFrame } from "../layout/PanelFrame.tsx";
import styles from "./ControlPanel.module.css";
import {
  derivePolicyUi,
  GROUPS,
  livePending,
  PENDING_MS,
  type PendingRequest,
  type PolicyButton,
  readPolicyState,
} from "./controlPolicy.ts";
import { useOptionalSlot } from "./hooks.ts";
import { paramChannel, readString } from "./panelParams.ts";
import type { PanelProps } from "./registry.tsx";
import { txReasonText } from "./txReason.ts";

export type Mode = "teleop" | "agent";

export function readMode(v: unknown): Mode | null {
  if (typeof v !== "object" || v === null) return null;
  const mode = readString((v as Record<string, unknown>).mode);
  return mode === "teleop" || mode === "agent" ? mode : null;
}

const keepFocus = (e: MouseEvent<HTMLButtonElement>): void => e.preventDefault();

export function ControlPanel({ spec, store, teleop }: PanelProps) {
  const modeCh = paramChannel(spec, "mode", 0);
  const policiesCh = paramChannel(spec, "policies", 1);
  const commandCh = paramChannel(spec, "command", 3);

  const mode = readMode(useOptionalSlot(store, modeCh)?.value);
  const policy = readPolicyState(useOptionalSlot(store, policiesCh)?.value);

  const [pending, setPending] = useState<PendingRequest[]>([]);
  const [pendingMode, setPendingMode] = useState<{ mode: Mode; at: number } | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    // Age out optimistic state the robot never confirmed.
    if (pending.length === 0 && pendingMode === null) return;
    const id = setTimeout(() => {
      const now = Date.now();
      setPending((p) => {
        const live = livePending(p, now);
        return live.length === p.length ? p : live;
      });
      setPendingMode((pm) => (pm !== null && now - pm.at >= PENDING_MS ? null : pm));
    }, PENDING_MS + 10);
    return () => clearTimeout(id);
  }, [pending, pendingMode]);

  const now = Date.now();
  const buttons = derivePolicyUi(policy, pending, now);
  const shownMode = pendingMode !== null && now - pendingMode.at < PENDING_MS &&
      pendingMode.mode !== mode
    ? pendingMode.mode
    : mode;
  const modePending = shownMode !== mode;

  const canSend = teleop !== undefined && commandCh !== undefined;
  const send = (data: Record<string, unknown>): boolean => {
    if (teleop === undefined || commandCh === undefined) return false;
    const result = teleop.tx(commandCh, data);
    if (!result.ok) {
      setError(`command not sent: ${txReasonText(result.reason)}`);
      return false;
    }
    setError(null);
    return true;
  };

  const setMode = (m: Mode): void => {
    if (send({ name: "set_mode", args: { mode: m } })) setPendingMode({ mode: m, at: Date.now() });
  };
  const request = (b: PolicyButton): void => {
    if (send({ name: "policy", args: { policy: b.name, action: b.action } })) {
      setPending((p) => [...p, { policy: b.name, action: b.action, at: Date.now() }]);
    }
  };
  return (
    <PanelFrame spec={spec}>
      <div className={styles.strip} data-testid={`control-${spec.id}`}>
        <div className={styles.segment} role="group" aria-label="mode">
          {(["teleop", "agent"] as const).map((m) => (
            <button
              key={m}
              type="button"
              className={styles.segmentButton}
              data-testid={`control-mode-${m}`}
              aria-pressed={shownMode === m}
              data-pending={(modePending && shownMode === m) || undefined}
              disabled={!canSend}
              onMouseDown={keepFocus}
              onClick={() => setMode(m)}
            >
              {m === "teleop" ? "Teleop" : "Agent"}
            </button>
          ))}
        </div>
        {GROUPS.map(({ kind, title }) => {
          const group = buttons.filter((b) => b.kind === kind);
          if (group.length === 0) return null;
          return (
            <div key={kind} className={styles.group} role="group" aria-label={title}>
              <span className={styles.groupTitle}>{title}</span>
              {group.map((b) => (
                <button
                  key={b.name}
                  type="button"
                  className={styles.policy}
                  data-testid={`policy-${b.name}`}
                  data-state={b.state}
                  title={b.reason ?? b.label}
                  disabled={!canSend || b.state === "unavailable" || b.state === "disabled"}
                  onMouseDown={keepFocus}
                  onClick={() => request(b)}
                >
                  {b.progress !== null && (
                    <span className={styles.progress} style={{ width: `${b.progress * 100}%` }} />
                  )}
                  <span className={styles.policyLabel}>{b.label}</span>
                </button>
              ))}
            </div>
          );
        })}
        {policy === null && <span className={styles.hint}>waiting for policy state...</span>}
        <div className={styles.chips}>
          {policy !== null && policy.variant !== "" && (
            <span className={styles.chip} data-testid="chip-variant">{policy.variant}</span>
          )}
          {policy !== null && policy.active !== null && (
            <span className={styles.chip} data-testid="chip-active">{policy.active}</span>
          )}
          {policy?.fallen === true && (
            <span className={styles.chipBad} data-testid="chip-fallen">fallen</span>
          )}
          {policy?.locked === true && (
            <span className={styles.chipWarn} data-testid="chip-locked">locked</span>
          )}
          {policy !== null && policy.last_error !== null && policy.last_error !== "" && (
            <span className={styles.chipBad} data-testid="chip-error" title={policy.last_error}>
              {policy.last_error}
            </span>
          )}
          {
            /* Nav state and its cancel live on the map, beside the goal
              marker and the path they describe; a second copy here read as
              two different controls. */
          }
          {error !== null && (
            <span className={styles.chipBad} role="alert" data-testid="control-error">{error}</span>
          )}
        </div>
      </div>
    </PanelFrame>
  );
}
