import * as d3 from "d3";
import * as React from "react";

import { Vector } from "../types";

interface GridLayerProps {
  width: number;
  height: number;
  origin: Vector;
  resolution: number;
  rows: number;
  cols: number;
}

const GridLayer = React.memo<GridLayerProps>(
  ({ width, height, origin, resolution, rows, cols }) => {
    const minX = origin.coords[0]!;
    const minY = origin.coords[1]!;
    const maxX = minX + cols * resolution;
    const maxY = minY + rows * resolution;

    const xScale = d3.scaleLinear().domain([minX, maxX]).range([0, width]);
    const yScale = d3.scaleLinear().domain([minY, maxY]).range([height, 0]);

    const gridSize = 1 / resolution;
    const gridLines = React.useMemo(() => {
      const lines = [];
      for (const x of d3.range(Math.ceil(minX / gridSize) * gridSize, maxX, gridSize)) {
        lines.push(
          <line
            key={`v-${x}`}
            x1={xScale(x)}
            y1={0}
            x2={xScale(x)}
            y2={height}
            stroke="#000"
            strokeWidth={0.5}
            opacity={0.25}
          />,
        );
      }
      for (const y of d3.range(Math.ceil(minY / gridSize) * gridSize, maxY, gridSize)) {
        lines.push(
          <line
            key={`h-${y}`}
            x1={0}
            y1={yScale(y)}
            x2={width}
            y2={yScale(y)}
            stroke="#000"
            strokeWidth={0.5}
            opacity={0.25}
          />,
        );
      }
      return lines;
    }, [minX, minY, maxX, maxY, gridSize, xScale, yScale, width, height]);

    const xAxisRef = React.useRef<SVGGElement>(null);
    const yAxisRef = React.useRef<SVGGElement>(null);

    React.useEffect(() => {
      if (xAxisRef.current) {
        const xAxis = d3.axisBottom(xScale).ticks(7);
        d3.select(xAxisRef.current).call(xAxis);
        d3.select(xAxisRef.current)
          .selectAll("line,path")
          .attr("stroke", "#ffffff")
          .attr("stroke-width", 1);
        d3.select(xAxisRef.current).selectAll("text").attr("fill", "#ffffff");
      }
      if (yAxisRef.current) {
        const yAxis = d3.axisLeft(yScale).ticks(7);
        d3.select(yAxisRef.current).call(yAxis);
        d3.select(yAxisRef.current)
          .selectAll("line,path")
          .attr("stroke", "#ffffff")
          .attr("stroke-width", 1);
        d3.select(yAxisRef.current).selectAll("text").attr("fill", "#ffffff");
      }
    }, [xScale, yScale]);

    const showOrigin = minX <= 0 && 0 <= maxX && minY <= 0 && 0 <= maxY;

    return (
      <>
        <g className="grid">{gridLines}</g>
        <g ref={xAxisRef} transform={`translate(0, ${height})`} />
        <g ref={yAxisRef} transform={`translate(0, 0)`} />
        {showOrigin && (
          <g className="origin-marker" transform={`translate(${xScale(0)}, ${yScale(0)})`}>
            <circle r={8} fill="none" stroke="#00e676" strokeWidth={1} opacity={0.5} />
            <circle r={4} fill="#00e676" opacity={0.9}>
              <title>World Origin (0,0)</title>
            </circle>
          </g>
        )}
      </>
    );
  },
);

GridLayer.displayName = "GridLayer";

export default GridLayer;
