import * as d3 from "d3";
import * as React from "react";

import { AppState } from "../types";
import VisualizerComponent from "./VisualizerComponent";

interface VisualizerWrapperProps {
  data: AppState;
  onWorldClick: (worldX: number, worldY: number) => void;
}

const VisualizerWrapper: React.FC<VisualizerWrapperProps> = ({ data, onWorldClick }) => {
  const containerRef = React.useRef<HTMLDivElement>(null);
  const lastClickTime = React.useRef(0);
  const clickThrottleMs = 150;

  const handleClick = React.useCallback(
    (event: React.MouseEvent) => {
      if (!data.costmap || !containerRef.current) {
        return;
      }

      event.stopPropagation();

      const now = Date.now();
      if (now - lastClickTime.current < clickThrottleMs) {
        console.log("Click throttled");
        return;
      }
      lastClickTime.current = now;

      const svgElement = containerRef.current.querySelector("svg");
      if (!svgElement) {
        return;
      }

      const svgRect = svgElement.getBoundingClientRect();
      const clickX = event.clientX - svgRect.left;
      const clickY = event.clientY - svgRect.top;

      const costmap = data.costmap;
      const {
        grid: { shape },
        origin,
        resolution,
      } = costmap;
      const rows = shape[0]!;
      const cols = shape[1]!;
      const width = svgRect.width;
      const height = svgRect.height;

      const axisMargin = { left: 60, bottom: 40 };
      const availableWidth = width - axisMargin.left;
      const availableHeight = height - axisMargin.bottom;

      const cell = Math.min(availableWidth / cols, availableHeight / rows);
      const gridW = cols * cell;
      const gridH = rows * cell;
      const offsetX = axisMargin.left + (availableWidth - gridW) / 2;
      const offsetY = (availableHeight - gridH) / 2;

      const xScale = d3
        .scaleLinear()
        .domain([origin.coords[0]!, origin.coords[0]! + cols * resolution])
        .range([offsetX, offsetX + gridW]);
      const yScale = d3
        .scaleLinear()
        .domain([origin.coords[1]!, origin.coords[1]! + rows * resolution])
        .range([offsetY + gridH, offsetY]);

      const worldX = xScale.invert(clickX);
      const worldY = yScale.invert(clickY);

      onWorldClick(worldX, worldY);
    },
    [data.costmap, onWorldClick],
  );

  return (
    <div ref={containerRef} style={{ width: "100%", height: "100%" }} onClick={handleClick}>
      <VisualizerComponent costmap={data.costmap} robotPose={data.robotPose} path={data.path} />
    </div>
  );
};

export default VisualizerWrapper;
