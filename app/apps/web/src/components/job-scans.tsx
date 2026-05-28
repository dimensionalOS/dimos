"use client";

import type { FrameAnalysis, ScanImage } from "@robomoo/shared";
import { Sparkles } from "lucide-react";
import { useEffect, useState } from "react";
import {
  Dialog,
  DialogContent,
  DialogTitle,
} from "@/components/ui/dialog";
import { AnalyzeRoomButton } from "@/components/scans-browser";
import { Skeleton } from "@/components/ui/skeleton";
import { rpcClient } from "@/lib/orpc";
import { useLiveQuery } from "@/lib/use-live-query";
import { cn } from "@/lib/utils";

// Run-scoped segmentation gallery for a job deliverable. Polls the run's frames
// and opens a lightbox with the VLM analysis (summary + per-object mask/crop
// toggle) — the "segmentation" payoff, scoped to this one hire.
export function JobScans({ run }: { run: string }) {
  const { data: scan, loading } = useLiveQuery(
    async () => {
      const rows = await rpcClient.frames.scans({ run });
      return rows[0] ?? null;
    },
    3000,
    [run],
  );
  const [selected, setSelected] = useState<ScanImage | null>(null);

  if (!scan) {
    return (
      <div className="flex flex-col gap-3">
        <p className="flex items-center gap-2 text-muted-foreground text-sm">
          {loading
            ? "Loading scan…"
            : "Waiting for scan frames — they stream in as the robot captures."}
        </p>
        <div className="flex gap-2">
          {[0, 1, 2, 3, 4].map((i) => (
            <Skeleton className="shimmer h-28 w-40 rounded-lg" key={i} />
          ))}
        </div>
      </div>
    );
  }

  return (
    <div className="flex flex-col gap-5">
      <div className="flex flex-wrap items-center justify-between gap-3">
        <p className="font-mono text-muted-foreground text-xs">
          {scan.positionCount} positions · {scan.imageCount} images
        </p>
        <AnalyzeRoomButton
          images={scan.positions.flatMap((pos) => pos.images)}
        />
      </div>
      {scan.positions.map((pos) => (
        <div className="flex flex-col gap-1.5" key={`${run}-${pos.position}`}>
          <span className="font-mono text-muted-foreground text-xs uppercase tracking-wider">
            Position {pos.position ?? "—"} · {pos.images.length} images
          </span>
          <div className="flex gap-2.5 overflow-x-auto pb-2">
            {pos.images.map((img) => (
              <button
                className="group relative shrink-0 cursor-zoom-in overflow-hidden rounded-lg border transition-colors hover:border-signal/50"
                key={img.id}
                onClick={() => setSelected(img)}
                type="button"
              >
                {/* biome-ignore lint/performance/noImgElement: presigned URL */}
                <img
                  alt={`angle ${img.angle ?? "?"}`}
                  className="h-28 w-40 object-cover transition-transform duration-200 group-hover:scale-105"
                  decoding="async"
                  loading="lazy"
                  src={img.url}
                />
                <span className="absolute right-1.5 bottom-1.5 rounded bg-black/70 px-1.5 py-0.5 font-mono text-[10px] text-white">
                  {img.angle ?? "?"}°
                </span>
                {img.analysisCount > 0 ? (
                  <span
                    className="absolute top-1.5 right-1.5 inline-flex items-center gap-1 rounded-full bg-signal/90 px-1.5 py-0.5 text-[9px] text-signal-foreground"
                    title={`${img.analysisCount} analysis(es)`}
                  >
                    <Sparkles size={9} /> {img.analysisCount}
                  </span>
                ) : null}
              </button>
            ))}
          </div>
        </div>
      ))}

      <Dialog
        onOpenChange={(o) => !o && setSelected(null)}
        open={selected !== null}
      >
        {selected ? <ScanLightbox image={selected} /> : null}
      </Dialog>
    </div>
  );
}

function ScanLightbox({ image }: { image: ScanImage }) {
  const [analyses, setAnalyses] = useState<FrameAnalysis[] | null>(null);
  const [override, setOverride] = useState<string | null>(null);

  useEffect(() => {
    setOverride(null);
    rpcClient.frames
      .analyses({ frameIds: [image.id] })
      .then(setAnalyses)
      .catch(() => setAnalyses([]));
  }, [image.id]);

  const latest = analyses?.[0] ?? null;

  return (
    <DialogContent className="max-w-4xl gap-3">
      <DialogTitle className="font-mono text-muted-foreground text-xs uppercase tracking-wider">
        Angle {image.angle ?? "?"}°
      </DialogTitle>
      {/* biome-ignore lint/performance/noImgElement: presigned URL */}
      <img
        alt=""
        className="max-h-[65vh] w-full rounded-lg border object-contain"
        src={override ?? image.url}
      />
      <div className="flex flex-col gap-2">
        {latest?.summary ? (
          <p className="text-sm leading-relaxed">{latest.summary}</p>
        ) : latest?.description ? (
          <p className="text-muted-foreground text-sm leading-relaxed">
            {latest.description}
          </p>
        ) : (
          <p className="text-muted-foreground text-sm">
            {analyses === null ? "Loading analysis…" : "No analysis yet."}
          </p>
        )}
        {latest && latest.objects.length > 0 ? (
          <div className="flex flex-wrap gap-1.5">
            <button
              className={cn(
                "rounded-md border px-2 py-0.5 text-xs transition-colors",
                override === null
                  ? "border-signal/50 bg-signal/10 text-signal"
                  : "hover:border-foreground/30",
              )}
              onClick={() => setOverride(null)}
              type="button"
            >
              frame
            </button>
            {latest.objects.map((o) => {
              const url = o.maskUrl ?? o.cropUrl ?? null;
              return (
                <button
                  className={cn(
                    "rounded-md border px-2 py-0.5 text-xs transition-colors disabled:opacity-40",
                    url && override === url
                      ? "border-signal/50 bg-signal/10 text-signal"
                      : "hover:border-foreground/30",
                  )}
                  disabled={!url}
                  key={o.id}
                  onClick={() => setOverride(url)}
                  type="button"
                >
                  {o.label ?? o.query ?? "object"}
                </button>
              );
            })}
          </div>
        ) : null}
      </div>
    </DialogContent>
  );
}
