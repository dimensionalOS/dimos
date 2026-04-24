import * as React from "react";

import Button from "./Button";

interface ActivationPanelProps {
  onArm: () => void;
  onDisarm: () => void;
  onSetDryRun: (enabled: boolean) => void;
  initialDryRun?: boolean;
}

/**
 * Dashboard control for arming/disarming a locomotion-policy task
 * (e.g. the G1 GR00T WBC).
 *
 * The panel is UI-only — it does not know whether the task actually
 * accepted the arm/disarm request.  The coordinator logs the
 * transition server-side; for a future iteration we could subscribe
 * to a state-echo stream and reflect the real machine-state here.
 *
 * Defaults to dry-run ON so hitting Arm on real hardware does NOT
 * immediately command motors — the operator toggles dry-run off
 * after visually verifying computed targets are sensible.
 */
export default function ActivationPanel({
  onArm,
  onDisarm,
  onSetDryRun,
  initialDryRun = true,
}: ActivationPanelProps): React.ReactElement {
  const [armed, setArmed] = React.useState(false);
  const [dryRun, setDryRun] = React.useState(initialDryRun);

  const handleArmToggle = () => {
    if (armed) {
      onDisarm();
      setArmed(false);
    } else {
      onArm();
      setArmed(true);
    }
  };

  const handleDryRunToggle = () => {
    const next = !dryRun;
    onSetDryRun(next);
    setDryRun(next);
  };

  return (
    <div
      style={{
        display: "flex",
        flexDirection: "column",
        gap: 6,
        padding: 6,
        border: "1px solid #444",
        borderRadius: 4,
        background: "#1a1a1a",
      }}
    >
      <div style={{ fontSize: 12, color: "#aaa" }}>Policy</div>
      <Button isActive={armed} onClick={handleArmToggle}>
        {armed ? "Disarm" : "Arm (ramp to default)"}
      </Button>
      <button
        onClick={handleDryRunToggle}
        style={{
          backgroundColor: dryRun ? "#ffa726" : "#555",
          color: "white",
          padding: "5px 10px",
          border: "none",
          borderRadius: "4px",
          cursor: "pointer",
          fontSize: "14px",
        }}
      >
        {dryRun ? "Dry Run ON (not sending)" : "Dry Run OFF (live)"}
      </button>
      <div style={{ fontSize: 11, color: "#888" }}>
        Arm ramps current pose → default over ~10 s. Dry run keeps policy computing but
        suppresses commands so you can verify targets in server logs first.
      </div>
    </div>
  );
}
