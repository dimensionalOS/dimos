import * as d3 from "d3";
import * as React from "react";

import { Costmap, Path, Vector } from "../types";
import CostmapLayer from "./CostmapLayer";
import PathLayer from "./PathLayer";
import VectorLayer from "./VectorLayer";

interface VisualizerComponentProps {
  costmap: Costmap | null;
  robotPose: Vector | null;
  path: Path | null;
}

const VisualizerComponent: React.FC<VisualizerComponentProps> = ({ costmap, robotPose, path }) => {
  const svgRef = React.useRef<SVGSVGElement>(null);
  const [dimensions, setDimensions] = React.useState({ width: 800, height: 600 });
  const { width, height } = dimensions;

  React.useEffect(() => {
    if (!svgRef.current?.parentElement) {
      return;
    }

    const updateDimensions = () => {
      const rect = svgRef.current?.parentElement?.getBoundingClientRect();
      if (rect) {
        setDimensions({ width: rect.width, height: rect.height });
      }
    };

    updateDimensions();
    const observer = new ResizeObserver(updateDimensions);
    observer.observe(svgRef.current.parentElement);

    return () => {
      observer.disconnect();
    };
  }, []);

  const { worldToPx } = React.useMemo(() => {
    if (!costmap) {
      return { worldToPx: undefined };
    }

    const {
      grid: { shape },
      origin,
      resolution,
    } = costmap;
    const rows = shape[0]!;
    const cols = shape[1]!;

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

    const worldToPxFn = (x: number, y: number): [number, number] => [xScale(x), yScale(y)];

    return { worldToPx: worldToPxFn };
  }, [costmap, width, height]);

  return (
    <div className="visualizer-container" style={{ width: "100%", height: "100%" }}>
      <svg
        ref={svgRef}
        width="100%"
        height="100%"
        viewBox={`0 0 ${width} ${height}`}
        preserveAspectRatio="xMidYMid meet"
        style={{
          backgroundColor: "black",
          pointerEvents: "none",
        }}
      >
        {costmap && <CostmapLayer costmap={costmap} width={width} height={height} />}
        {path && worldToPx && <PathLayer path={path} worldToPx={worldToPx} />}
        {robotPose && worldToPx && (
          <VectorLayer vector={robotPose} label="robot" worldToPx={worldToPx} />
        )}
      </svg>
    </div>
  );
};

export default React.memo(VisualizerComponent);
