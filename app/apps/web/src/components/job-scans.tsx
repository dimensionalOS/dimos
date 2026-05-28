"use client";

import type { FrameAnalysis, ScanImage, ScanRun } from "@robomoo/shared";
import { useEffect, useState } from "react";
import { rpcClient } from "@/lib/orpc";

// Run-scoped segmentation gallery for a job deliverable. Polls the run's frames
// and opens a lightbox with the VLM analysis (summary + per-object mask/crop
// toggle) — the "segmentation" payoff, scoped to this one hire.
export function JobScans({ run }: { run: string }) {
  const [scan, setScan] = useState<ScanRun | null>(null);
  const [selected, setSelected] = useState<ScanImage | null>(null);

  useEffect(() => {
    let active = true;
    const load = async () => {
      try {
        const rows = await rpcClient.frames.scans({ run });
        if (active) setScan(rows[0] ?? null);
      } catch {
        /* ignore */
      }
    };
    load();
    const t = setInterval(load, 5000);
    return () => {
      active = false;
      clearInterval(t);
    };
  }, [run]);

  if (!scan) {
    return (
      <p className="text-muted-foreground text-sm">Waiting for scan frames…</p>
    );
  }

  return (
    <div className="flex flex-col gap-4">
      <p className="text-muted-foreground text-xs">
        {scan.positionCount} positions · {scan.imageCount} images
      </p>
      {scan.positions.map((pos) => (
        <div className="flex flex-col gap-1" key={`${run}-${pos.position}`}>
          <span className="text-muted-foreground text-xs">
            Position {pos.position ?? "—"} · {pos.images.length} images
          </span>
          <div className="flex gap-2 overflow-x-auto pb-2">
            {pos.images.map((img) => (
              <button
                className="relative shrink-0 cursor-zoom-in"
                key={img.id}
                onClick={() => setSelected(img)}
                type="button"
              >
                {/* biome-ignore lint/performance/noImgElement: presigned URL */}
                <img
                  alt={`angle ${img.angle ?? "?"}`}
                  className="h-24 w-32 rounded border object-cover transition-opacity hover:opacity-90"
                  decoding="async"
                  loading="lazy"
                  src={img.url}
                />
                <span className="absolute right-0 bottom-0 bg-black/60 px-1 text-[10px] text-white">
                  {img.angle ?? "?"}°
                </span>
                {img.analysisCount > 0 ? (
                  <span
                    className="absolute top-1 right-1 h-2 w-2 rounded-full bg-emerald-400 ring-1 ring-emerald-900/40"
                    title={`${img.analysisCount} analysis(es)`}
                  />
                ) : null}
              </button>
            ))}
          </div>
        </div>
      ))}
      {selected ? (
        <ScanLightbox image={selected} onClose={() => setSelected(null)} />
      ) : null}
    </div>
  );
}

function ScanLightbox({
  image,
  onClose,
}: {
  image: ScanImage;
  onClose: () => void;
}) {
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
    // biome-ignore lint/a11y/useKeyWithClickEvents: simple click-to-dismiss lightbox
    // biome-ignore lint/a11y/noStaticElementInteractions: simple click-to-dismiss lightbox
    <div
      className="fixed inset-0 z-50 flex flex-col items-center justify-start gap-3 overflow-y-auto bg-black/80 p-4"
      onClick={onClose}
    >
      {/* biome-ignore lint/performance/noImgElement: presigned URL */}
      <img
        alt=""
        className="max-h-[70vh] max-w-full object-contain"
        src={override ?? image.url}
      />
      <div
        className="flex w-full max-w-3xl flex-col gap-2 text-white"
        onClick={(e) => e.stopPropagation()}
        onKeyDown={(e) => e.stopPropagation()}
      >
        {latest?.summary ? (
          <p className="text-white/90 text-sm">{latest.summary}</p>
        ) : latest?.description ? (
          <p className="text-white/80 text-sm">{latest.description}</p>
        ) : (
          <p className="text-white/60 text-sm">
            {analyses === null ? "Loading analysis…" : "No analysis yet."}
          </p>
        )}
        {latest && latest.objects.length > 0 ? (
          <div className="flex flex-wrap gap-1">
            <button
              className="rounded border border-white/30 bg-white/5 px-2 py-0.5 text-xs hover:bg-white/15"
              onClick={() => setOverride(null)}
              type="button"
            >
              frame
            </button>
            {latest.objects.map((o) => (
              <button
                className="rounded border border-white/30 bg-white/5 px-2 py-0.5 text-xs hover:bg-white/15"
                disabled={!o.maskUrl && !o.cropUrl}
                key={o.id}
                onClick={() => setOverride(o.maskUrl ?? o.cropUrl ?? null)}
                type="button"
              >
                {o.label ?? o.query ?? "object"}
              </button>
            ))}
          </div>
        ) : null}
      </div>
      <span className="text-white/50 text-xs">click the background to close</span>
    </div>
  );
}
