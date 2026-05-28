"use client";

import { useState } from "react";
import { ConsolePanel } from "@/components/console-panel";
import { FramesGallery } from "@/components/frames-gallery";
import { MapView } from "@/components/map-view";
import { RobotConnection } from "@/components/robot-connection";
import { RobotControls } from "@/components/robot-controls";
import { ScansBrowser } from "@/components/scans-browser";
import { SplatViewer } from "@/components/splat-viewer";
import { cn } from "@/lib/utils";

const TABS = [
  { key: "frames", label: "Frames" },
  { key: "scans", label: "Scans" },
  { key: "splats", label: "Splats" },
] as const;
type TabKey = (typeof TABS)[number]["key"];

export default function ConsolePage() {
  const [tab, setTab] = useState<TabKey>("frames");

  return (
    <main className="mx-auto flex max-w-5xl flex-col gap-8 px-4 py-12">
      <header className="flex flex-col gap-2">
        <h1 className="font-bold font-display text-3xl tracking-tight">
          Robot console
        </h1>
        <p className="text-muted-foreground text-sm">
          Drive the robot, point it at your own hardware, and watch what it sees
          — live.
        </p>
      </header>

      <ConsolePanel title="Robot connection + control">
        <div className="flex flex-col gap-5 p-4">
          <RobotConnection />
          <div className="border-t" />
          <RobotControls />
        </div>
      </ConsolePanel>

      <ConsolePanel live title="Live map · robot path">
        <div className="p-4">
          <MapView />
        </div>
      </ConsolePanel>

      <section className="flex flex-col gap-4">
        <div className="flex w-fit items-center gap-1 rounded-lg border bg-card p-1">
          {TABS.map((t) => (
            <button
              className={cn(
                "rounded-md px-3 py-1.5 font-medium text-sm transition-colors",
                tab === t.key
                  ? "bg-signal/15 text-signal"
                  : "text-muted-foreground hover:text-foreground",
              )}
              key={t.key}
              onClick={() => setTab(t.key)}
              type="button"
            >
              {t.label}
            </button>
          ))}
        </div>
        {tab === "frames" ? (
          <FramesGallery />
        ) : tab === "scans" ? (
          <ScansBrowser />
        ) : (
          <SplatViewer />
        )}
      </section>
    </main>
  );
}
