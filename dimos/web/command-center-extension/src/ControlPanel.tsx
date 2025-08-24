import * as React from "react";

interface ControlPanelProps {
  onStartExplore: () => void;
  onStopExplore: () => void;
}

export default function ControlPanel({
  onStartExplore,
  onStopExplore,
}: ControlPanelProps): React.ReactElement {
  const [exploring, setExploring] = React.useState(false);

  return (
    <div style={{ width: "100%", padding: 5 }}>
      {exploring ? (
        <button
          onClick={() => {
            onStopExplore();
            setExploring(false);
          }}
        >
          Stop Exploration
        </button>
      ) : (
        <button
          onClick={() => {
            onStartExplore();
            setExploring(true);
          }}
        >
          Start Exploration
        </button>
      )}
    </div>
  );
}
