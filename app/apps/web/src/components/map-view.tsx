"use client";

import type { Frame, MapSnapshot } from "@robomoo/shared";
import { useEffect, useState } from "react";
import { rpcClient } from "@/lib/orpc";

// Renders the latest robot map snapshot (PNG) with a pin per captured frame.
// World→pixel: col = (x - originX)/resolution, row = (y - originY)/resolution.
// An SVG viewBox sized to the grid lets the browser scale pins with the image.
export function MapView() {
  const [map, setMap] = useState<MapSnapshot | null>(null);
  const [frames, setFrames] = useState<Frame[]>([]);
  const [selected, setSelected] = useState<Frame | null>(null);

  useEffect(() => {
    let active = true;
    const load = async () => {
      try {
        const [m, f] = await Promise.all([
          rpcClient.map.latest(),
          rpcClient.frames.list(),
        ]);
        if (active) {
          setMap(m);
          setFrames(f);
        }
      } catch {
        /* keep last good state */
      }
    };
    load();
    const t = setInterval(load, 3000);
    return () => {
      active = false;
      clearInterval(t);
    };
  }, []);

  if (!map) {
    return (
      <p className="text-muted-foreground text-sm">
        No map snapshot yet. The robot POSTs one to <code>/api/robot/map</code>.
      </p>
    );
  }

  const pins = frames.filter((f) => f.poseX !== null && f.poseY !== null);
  const r = Math.max(map.width, map.height) / 90; // pin radius in grid cells

  return (
    <div className="flex flex-col gap-4">
      <div className="relative w-full overflow-hidden rounded-md border bg-black">
        {/* biome-ignore lint/performance/noImgElement: presigned URL, plain img is simplest */}
        <img alt="robot map" className="block w-full" src={map.imageUrl} />
        <svg
          aria-label="capture markers"
          className="absolute inset-0 h-full w-full"
          preserveAspectRatio="none"
          viewBox={`0 0 ${map.width} ${map.height}`}
        >
          {pins.map((f) => {
            const cx = ((f.poseX as number) - map.originX) / map.resolution;
            const cy = ((f.poseY as number) - map.originY) / map.resolution;
            return (
              <circle
                className="cursor-pointer"
                cx={cx}
                cy={cy}
                fill={selected?.id === f.id ? "#22c55e" : "#ef4444"}
                key={f.id}
                onClick={() => setSelected(f)}
                r={r}
                stroke="white"
                strokeWidth={r / 4}
              >
                <title>{f.label ?? f.note ?? "capture"}</title>
              </circle>
            );
          })}
        </svg>
      </div>

      {selected ? (
        <div className="flex flex-col gap-1">
          {/* biome-ignore lint/performance/noImgElement: presigned URL */}
          <img
            alt={selected.label ?? "capture"}
            className="max-h-80 rounded-md border object-contain"
            src={selected.imageUrl}
          />
          <span className="text-muted-foreground text-xs">
            {selected.label ?? selected.note ?? "capture"} ·{" "}
            {new Date(selected.createdAt).toLocaleString()}
          </span>
        </div>
      ) : (
        <p className="text-muted-foreground text-sm">
          {pins.length} capture{pins.length === 1 ? "" : "s"} on the map — click a
          pin to view the photo.
        </p>
      )}
    </div>
  );
}
