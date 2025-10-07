import * as React from "react";

import Button from "./Button";

interface GpsButtonProps {
  onUseGps: () => void;
  onUseCostmap: () => void;
}

export default function GpsButton({
  onUseGps,
  onUseCostmap,
}: GpsButtonProps): React.ReactElement {
  const [gps, setGps] = React.useState(false);

  return (
    <div>
      {gps ? (
        <Button
          onClick={() => {
            onUseCostmap();
            setGps(false);
          }}
          isActive={true}
        >
          Use Costmap
        </Button>
      ) : (
        <Button
          onClick={() => {
            onUseGps();
            setGps(true);
          }}
          isActive={false}
        >
          Use GPS
        </Button>
      )}
    </div>
  );
}
