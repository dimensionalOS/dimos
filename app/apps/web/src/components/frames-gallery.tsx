"use client";

import type { Frame } from "@robomoo/shared";
import { useEffect, useState } from "react";
import { Card, CardContent } from "@/components/ui/card";
import { rpcClient } from "@/lib/orpc";

// Live-ish gallery of robot-captured frames. Polls every 3s so new captures
// (e.g. after "take a picture") show up without a manual refresh.
export function FramesGallery() {
  const [frames, setFrames] = useState<Frame[]>([]);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let active = true;
    const load = async () => {
      try {
        const next = await rpcClient.frames.list();
        if (active) {
          setFrames(next);
          setError(null);
        }
      } catch (e) {
        if (active) setError(e instanceof Error ? e.message : "failed to load");
      }
    };
    load();
    const t = setInterval(load, 3000);
    return () => {
      active = false;
      clearInterval(t);
    };
  }, []);

  if (error) {
    return <p className="text-destructive text-sm">Error: {error}</p>;
  }
  if (frames.length === 0) {
    return (
      <p className="text-muted-foreground text-sm">
        No frames yet. POST one to <code>/api/robot/frame</code> or ask the robot
        to take a picture.
      </p>
    );
  }

  return (
    <div className="grid grid-cols-2 gap-3 sm:grid-cols-3">
      {frames.map((f) => (
        <Card key={f.id} className="overflow-hidden">
          <CardContent className="p-0">
            {/* biome-ignore lint/performance/noImgElement: presigned URLs rotate; plain img is simplest */}
            <img
              alt={f.note ?? "robot frame"}
              className="aspect-video w-full object-cover"
              src={f.imageUrl}
            />
            <div className="flex flex-col gap-0.5 p-2">
              {f.note ? <span className="text-xs">{f.note}</span> : null}
              <time className="text-muted-foreground text-xs">
                {new Date(f.createdAt).toLocaleString()}
              </time>
            </div>
          </CardContent>
        </Card>
      ))}
    </div>
  );
}
