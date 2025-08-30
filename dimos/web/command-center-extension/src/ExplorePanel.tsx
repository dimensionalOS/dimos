import * as React from "react";

import Button from "./Button";

interface ControlPanelProps {
  onStartExplore: () => void;
  onStopExplore: () => void;
}

export default function ExplorePanel({
  onStartExplore,
  onStopExplore,
}: ControlPanelProps): React.ReactElement {
  const [exploring, setExploring] = React.useState(false);

  return (
    <div>
      {exploring ? (
        <Button
          onClick={() => {
            onStopExplore();
            setExploring(false);
          }}
          isActive={true}
        >
          Stop Exploration
        </Button>
      ) : (
        <Button
          onClick={() => {
            onStartExplore();
            setExploring(true);
          }}
          isActive={false}
        >
          Start Exploration
        </Button>
      )}
    </div>
  );
}
