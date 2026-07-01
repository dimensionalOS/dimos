// Dependency-free SVG over a rolling number[] window; auto-scales to the window max; needs ≥2 samples; polyline is enough at ~4 Hz (no canvas).
interface SparklineProps {
  data: number[];
  width?: number;
  height?: number;
  color?: string;
  /** Fixed max for the y-axis; omit to auto-scale to the window's peak. */
  max?: number;
}

export function Sparkline({ data, width = 120, height = 28, color = "var(--signal)", max }: SparklineProps) {
  const n = data.length;
  const peak = max ?? Math.max(1, ...data);
  const pts = n >= 2
    ? data
      .map((v, i) => `${(i / (n - 1)) * width},${height - (Math.max(0, v) / peak) * height}`)
      .join(" ")
    : "";
  return (
    <svg
      width={width}
      height={height}
      viewBox={`0 0 ${width} ${height}`}
      preserveAspectRatio="none"
      style={{ display: "block" }}
    >
      {pts && (
        <polyline
          points={pts}
          fill="none"
          stroke={color}
          strokeWidth={1.5}
          strokeLinejoin="round"
          strokeLinecap="round"
        />
      )}
    </svg>
  );
}
