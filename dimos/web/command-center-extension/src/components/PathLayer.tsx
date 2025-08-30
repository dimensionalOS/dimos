import * as d3 from "d3";
import * as React from "react";

import { Path } from "../types";

interface PathLayerProps {
  path: Path;
  worldToPx: (x: number, y: number) => [number, number];
}

const PathLayer = React.memo<PathLayerProps>(({ path, worldToPx }) => {
  const points = React.useMemo(
    () => path.coords.map(([x, y]) => worldToPx(x, y)),
    [path.coords, worldToPx],
  );

  const pathData = React.useMemo(() => {
    const line = d3.line();
    return line(points);
  }, [points]);

  const gradientId = React.useMemo(() => `path-gradient-${Date.now()}`, []);

  if (path.coords.length < 2) {
    return null;
  }

  return (
    <>
      <defs>
        <linearGradient
          id={gradientId}
          gradientUnits="userSpaceOnUse"
          x1={points[0]![0]}
          y1={points[0]![1]}
          x2={points[points.length - 1]![0]}
          y2={points[points.length - 1]![1]}
        >
          <stop offset="0%" stopColor="#ff3333" />
          <stop offset="100%" stopColor="#ff3333" />
        </linearGradient>
      </defs>
      <path
        d={pathData ?? ""}
        fill="none"
        stroke={`url(#${gradientId})`}
        strokeWidth={5}
        strokeLinecap="round"
        opacity={0.9}
      />
    </>
  );
});

PathLayer.displayName = "PathLayer";

export default PathLayer;
