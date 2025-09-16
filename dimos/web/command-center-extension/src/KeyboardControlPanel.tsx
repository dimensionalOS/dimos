import * as React from "react";

import Button from "./Button";

interface KeyboardControlPanelProps {
  onSendMoveCommand: (linear: [number, number, number], angular: [number, number, number]) => void;
  onStopMoveCommand: () => void;
}

const linearSpeed = 0.5;
const angularSpeed = 0.8;
const publishRate = 10.0; // Hz

function calculateVelocities(keys: Set<string>) {
  let linearX = 0.0;
  let linearY = 0.0;
  let angularY = 0.0;
  let angularZ = 0.0;

  let speedMultiplier = 1.0;
  if (keys.has("Shift")) {
    speedMultiplier = 2.0; // Boost mode
  } else if (keys.has("Control")) {
    speedMultiplier = 0.5; // Slow mode
  }

  // Check for stop command (space)
  if (keys.has(" ")) {
    return { linearX: 0, linearY: 0, angularY: 0, angularZ: 0 };
  }

  // Linear X (forward/backward) - W/S
  if (keys.has("w")) {
    linearX = linearSpeed * speedMultiplier;
  } else if (keys.has("s")) {
    linearX = -linearSpeed * speedMultiplier;
  }

  // Angular Z (yaw/turn) - A/D
  if (keys.has("a")) {
    angularZ = angularSpeed * speedMultiplier;
  } else if (keys.has("d")) {
    angularZ = -angularSpeed * speedMultiplier;
  }

  // Linear Y (strafe) - Left/Right arrows
  if (keys.has("ArrowLeft")) {
    linearY = linearSpeed * speedMultiplier;
  } else if (keys.has("ArrowRight")) {
    linearY = -linearSpeed * speedMultiplier;
  }

  // Angular Y (pitch) - Up/Down arrows
  if (keys.has("ArrowUp")) {
    angularY = angularSpeed * speedMultiplier;
  } else if (keys.has("ArrowDown")) {
    angularY = -angularSpeed * speedMultiplier;
  }

  return { linearX, linearY, angularY, angularZ };
}

export default function KeyboardControlPanel({
  onSendMoveCommand,
  onStopMoveCommand,
}: KeyboardControlPanelProps): React.ReactElement {
  const [isActive, setIsActive] = React.useState(false);
  const keysPressed = React.useRef<Set<string>>(new Set());
  const intervalRef = React.useRef<NodeJS.Timeout | null>(null);

  const handleKeyDown = React.useCallback((event: KeyboardEvent) => {
    // Prevent default for arrow keys and space to avoid scrolling
    if (["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", " "].includes(event.key)) {
      event.preventDefault();
    }

    const normalizedKey = event.key.length === 1 ? event.key.toLowerCase() : event.key;
    keysPressed.current.add(normalizedKey);
  }, []);

  const handleKeyUp = React.useCallback((event: KeyboardEvent) => {
    const normalizedKey = event.key.length === 1 ? event.key.toLowerCase() : event.key;
    keysPressed.current.delete(normalizedKey);
  }, []);

  // Start/stop keyboard control
  React.useEffect(() => {
    keysPressed.current.clear();

    if (!isActive) {
      return undefined;
    }

    document.addEventListener("keydown", handleKeyDown);
    document.addEventListener("keyup", handleKeyUp);

    // Start publishing loop
    intervalRef.current = setInterval(() => {
      const velocities = calculateVelocities(keysPressed.current);

      onSendMoveCommand(
        [velocities.linearX, velocities.linearY, 0],
        [0, velocities.angularY, velocities.angularZ],
      );
    }, 1000 / publishRate);

    return () => {
      document.removeEventListener("keydown", handleKeyDown);
      document.removeEventListener("keyup", handleKeyUp);

      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }

      keysPressed.current.clear();
      onStopMoveCommand();
    };
  }, [isActive, handleKeyDown, handleKeyUp, onSendMoveCommand, onStopMoveCommand]);

  const toggleKeyboardControl = () => {
    if (isActive) {
      keysPressed.current.clear();
      setIsActive(false);
    } else {
      setIsActive(true);
    }
  };

  React.useEffect(() => {
    const handleBlur = () => {
      if (isActive) {
        keysPressed.current.clear();
        setIsActive(false);
      }
    };

    const handleFocus = () => {
      // Clear keys when window regains focus to avoid stuck keys
      keysPressed.current.clear();
    };

    window.addEventListener("blur", handleBlur);
    window.addEventListener("focus", handleFocus);

    return () => {
      window.removeEventListener("blur", handleBlur);
      window.removeEventListener("focus", handleFocus);
    };
  }, [isActive]);

  return (
    <div>
      {isActive && (
        <div style={{ marginTop: 10, fontSize: 12, color: "#666" }}>
          <div>Controls:</div>
          <div>W/S: Forward/Backward | A/D: Turn</div>
          <div>Arrows: Strafe/Pitch | Space: Stop</div>
          <div>Shift: Boost | Ctrl: Slow</div>
        </div>
      )}
      <Button isActive={isActive} onClick={toggleKeyboardControl}>
        {isActive ? "Stop Keyboard Control" : "Start Keyboard Control"}
      </Button>
    </div>
  );
}
