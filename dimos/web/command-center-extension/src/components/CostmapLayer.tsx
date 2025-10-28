import * as d3 from "d3";
import * as React from "react";

import { Costmap } from "../types";
import GridLayer from "./GridLayer";

interface CostmapLayerProps {
  costmap: Costmap;
  width: number;
  height: number;
}

const CostmapLayer = React.memo<CostmapLayerProps>(({ costmap, width, height }) => {
  const canvasRef = React.useRef<HTMLCanvasElement>(null);
  const { grid, origin, resolution } = costmap;
  const rows = Math.max(1, grid.shape[0] || 1);
  const cols = Math.max(1, grid.shape[1] || 1);

  const axisMargin = { left: 60, bottom: 40 };
  const availableWidth = Math.max(1, width - axisMargin.left);
  const availableHeight = Math.max(1, height - axisMargin.bottom);

  const cell = Math.max(0, Math.min(availableWidth / cols, availableHeight / rows));
  const gridW = Math.max(0, cols * cell);
  const gridH = Math.max(0, rows * cell);
  const offsetX = axisMargin.left + (availableWidth - gridW) / 2;
  const offsetY = (availableHeight - gridH) / 2;

  // Pre-compute color lookup table using exact D3 colors (computed once on mount)
  const colorLookup = React.useMemo(() => {
    const lookup = new Uint8ClampedArray(256 * 3); // RGB values for -1 to 254 (255 total values)

    const customColorScale = (t: number) => {
      if (t === 0) {
        return "black";
      }
      if (t < 0) {
        return "#2d2136";
      }
      if (t > 0.95) {
        return "#000000";
      }

      const color = d3.interpolateTurbo(t * 2 - 1);
      const hsl = d3.hsl(color);
      hsl.s *= 0.75;
      return hsl.toString();
    };

    const colour = d3.scaleSequential(customColorScale).domain([-1, 100]);

    // Pre-compute all 256 possible color values
    for (let i = 0; i < 256; i++) {
      const value = i === 255 ? -1 : i;
      const colorStr = colour(value);
      const c = d3.color(colorStr);

      if (c) {
        const rgb = c as d3.RGBColor;
        lookup[i * 3] = rgb.r;
        lookup[i * 3 + 1] = rgb.g;
        lookup[i * 3 + 2] = rgb.b;
      } else {
        lookup[i * 3] = 0;
        lookup[i * 3 + 1] = 0;
        lookup[i * 3 + 2] = 0;
      }
    }

    return lookup;
  }, []);

  React.useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }

    // Validate grid data length matches dimensions
    const expectedLength = rows * cols;
    if (grid.data.length !== expectedLength) {
      console.warn(
        `Grid data length mismatch: expected ${expectedLength}, got ${grid.data.length} (rows=${rows}, cols=${cols})`
      );
    }

    canvas.width = cols;
    canvas.height = rows;
    const ctx = canvas.getContext("2d");
    if (!ctx) {
      return;
    }

    const img = ctx.createImageData(cols, rows);
    const data = grid.data;
    const imgData = img.data;

    for (let i = 0; i < data.length && i < rows * cols; i++) {
      const row = Math.floor(i / cols);
      const col = i % cols;
      const invertedRow = rows - 1 - row;
      const srcIdx = invertedRow * cols + col;

      if (srcIdx < 0 || srcIdx >= data.length) {
        continue;
      }

      const value = data[i]!;
      // Map value to lookup index (handle -1 -> 255 mapping)
      const lookupIdx = value === -1 ? 255 : Math.min(254, Math.max(0, value));

      const o = srcIdx * 4;
      if (o < 0 || o + 3 >= imgData.length) {
        continue;
      }

      // Use pre-computed colors from lookup table
      const colorOffset = lookupIdx * 3;
      imgData[o] = colorLookup[colorOffset]!;
      imgData[o + 1] = colorLookup[colorOffset + 1]!;
      imgData[o + 2] = colorLookup[colorOffset + 2]!;
      imgData[o + 3] = 255;
    }

    ctx.putImageData(img, 0, 0);
  }, [grid.data, cols, rows, colorLookup]);

  return (
    <g transform={`translate(${offsetX}, ${offsetY})`}>
      <foreignObject width={gridW} height={gridH}>
        <div
          style={{
            width: "100%",
            height: "100%",
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
          }}
        >
          <canvas
            ref={canvasRef}
            style={{
              width: "100%",
              height: "100%",
              objectFit: "contain",
              backgroundColor: "black",
            }}
          />
        </div>
      </foreignObject>
      <GridLayer
        width={gridW}
        height={gridH}
        origin={origin}
        resolution={resolution}
        rows={rows}
        cols={cols}
      />
    </g>
  );
});

CostmapLayer.displayName = "CostmapLayer";

export default CostmapLayer;
