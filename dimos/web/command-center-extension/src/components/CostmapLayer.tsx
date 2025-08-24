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
  const rows = grid.shape[0]!;
  const cols = grid.shape[1]!;

  const axisMargin = { left: 60, bottom: 40 };
  const availableWidth = width - axisMargin.left;
  const availableHeight = height - axisMargin.bottom;

  const cell = Math.min(availableWidth / cols, availableHeight / rows);
  const gridW = cols * cell;
  const gridH = rows * cell;
  const offsetX = axisMargin.left + (availableWidth - gridW) / 2;
  const offsetY = (availableHeight - gridH) / 2;

  React.useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }

    canvas.width = cols;
    canvas.height = rows;
    const ctx = canvas.getContext("2d");
    if (!ctx) {
      return;
    }

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
    const img = ctx.createImageData(cols, rows);
    const data = grid.data;

    for (let i = 0; i < data.length; i++) {
      const row = Math.floor(i / cols);
      const col = i % cols;
      const invertedRow = rows - 1 - row;
      const srcIdx = invertedRow * cols + col;
      const value = data[i]!;
      const c = d3.color(colour(value));
      if (!c) {
        continue;
      }

      const o = srcIdx * 4;
      const rgb = c as d3.RGBColor;
      img.data[o] = rgb.r;
      img.data[o + 1] = rgb.g;
      img.data[o + 2] = rgb.b;
      img.data[o + 3] = 255;
    }
    ctx.putImageData(img, 0, 0);
  }, [grid.data, cols, rows]);

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
