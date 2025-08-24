import * as React from "react";

import { Vector } from "../types";

interface VectorLayerProps {
  vector: Vector;
  label: string;
  worldToPx: (x: number, y: number) => [number, number];
}

const VectorLayer = React.memo<VectorLayerProps>(({ vector, label, worldToPx }) => {
  const [cx, cy] = worldToPx(vector.coords[0]!, vector.coords[1]!);
  const text = `${label} (${vector.coords[0]!.toFixed(2)}, ${vector.coords[1]!.toFixed(2)})`;

  return (
    <>
      <g className="vector-marker" transform={`translate(${cx}, ${cy})`}>
        <circle r={10} fill="none" stroke="red" strokeWidth={1} opacity={0.9} />
        <circle r={6} fill="red" />
      </g>
      <g>
        <rect
          x={cx + 24}
          y={cy + 14}
          width={text.length * 7}
          height={18}
          fill="black"
          stroke="black"
          opacity={0.75}
        />
        <text x={cx + 25} y={cy + 25} fontSize="1em" fill="white">
          {text}
        </text>
      </g>
    </>
  );
});

VectorLayer.displayName = "VectorLayer";

export default VectorLayer;
