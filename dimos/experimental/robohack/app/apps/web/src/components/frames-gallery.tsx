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
  const [selected, setSelected] = useState<Frame | null>(null);

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
    <>
      <div className="grid grid-cols-2 gap-3 sm:grid-cols-3">
        {frames.map((f) => (
          <Card className="overflow-hidden" key={f.id}>
            <CardContent className="p-0">
              {/* biome-ignore lint/performance/noImgElement: presigned URLs rotate; plain img is simplest */}
              <img
                alt={f.note ?? "robot frame"}
                className="aspect-video w-full cursor-zoom-in object-cover transition-opacity hover:opacity-90"
                onClick={() => setSelected(f)}
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

      {selected ? (
        // biome-ignore lint/a11y/useKeyWithClickEvents: simple click-to-dismiss lightbox
        <div
          className="fixed inset-0 z-50 flex flex-col items-center justify-center gap-3 bg-black/80 p-4"
          onClick={() => setSelected(null)}
        >
          {/* biome-ignore lint/performance/noImgElement: presigned URL, plain img */}
          <img
            alt={selected.note ?? "robot frame"}
            className="max-h-[85vh] max-w-full object-contain"
            src={selected.imageUrl}
          />
          <div className="flex items-center gap-3 text-white/80 text-xs">
            {selected.note ? <span>{selected.note}</span> : null}
            <span>{new Date(selected.createdAt).toLocaleString()}</span>
            <a
              className="underline"
              href={selected.imageUrl}
              onClick={(e) => e.stopPropagation()}
              rel="noopener noreferrer"
              target="_blank"
            >
              open original ↗
            </a>
          </div>
          <span className="text-white/50 text-xs">click anywhere to close</span>
        </div>
      ) : null}
    </>
  );
}
