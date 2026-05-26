"use client";

import type { Splat } from "@robomoo/shared";
import dynamic from "next/dynamic";
import { useEffect, useMemo, useState } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { rpcClient } from "@/lib/orpc";

// Load the WebGL canvas only in the browser — Spark needs window/WebGL/WASM, so
// it must never run during SSR. dynamic(ssr:false) lives in a client component.
const SplatCanvas = dynamic(() => import("./splat-canvas"), {
  ssr: false,
  loading: () => (
    <div className="flex h-full w-full items-center justify-center text-muted-foreground text-sm">
      Initializing viewer…
    </div>
  ),
});

type Scene = { id: string; name: string; url: string; kind: "demo" | "robot" };

// Bundled demo splats (served from /public/splats — works fully offline). These
// are the official Spark sample assets; swap in real robot captures via the
// /api/robot/splat ingest endpoint, which then show up in the list below.
const DEFAULT_SCENE: Scene = {
  id: "demo-butterfly",
  name: "Butterfly (demo)",
  url: "/splats/butterfly.spz",
  kind: "demo",
};
const DEMO_SCENES: Scene[] = [
  DEFAULT_SCENE,
  { id: "demo-cat", name: "Cat (demo)", url: "/splats/cat.spz", kind: "demo" },
  { id: "demo-penguin", name: "Penguin (demo)", url: "/splats/penguin.spz", kind: "demo" },
];

export function SplatViewer() {
  const [robotSplats, setRobotSplats] = useState<Splat[]>([]);
  const [selectedId, setSelectedId] = useState<string>(DEFAULT_SCENE.id);
  const [flip, setFlip] = useState(false);
  const [urlInput, setUrlInput] = useState("");
  const [customScene, setCustomScene] = useState<Scene | null>(null);
  const [listError, setListError] = useState<string | null>(null);

  // Pull any real splats produced by the reconstruction pipeline.
  useEffect(() => {
    let active = true;
    rpcClient.splats
      .list()
      .then((rows) => {
        if (active) setRobotSplats(rows);
      })
      .catch((e) => {
        if (active)
          setListError(e instanceof Error ? e.message : "failed to load splats");
      });
    return () => {
      active = false;
    };
  }, []);

  const scenes: Scene[] = useMemo(() => {
    const robot: Scene[] = robotSplats.map((s) => ({
      id: s.id,
      name: s.name ?? `${s.id} (${s.format})`,
      url: s.splatUrl,
      kind: "robot",
    }));
    return [...robot, ...DEMO_SCENES, ...(customScene ? [customScene] : [])];
  }, [robotSplats, customScene]);

  const selected =
    scenes.find((s) => s.id === selectedId) ?? DEFAULT_SCENE;

  const loadUrl = () => {
    const u = urlInput.trim();
    if (!u) return;
    const scene: Scene = { id: "custom", name: "Pasted URL", url: u, kind: "robot" };
    setCustomScene(scene);
    setSelectedId("custom");
  };

  return (
    <div className="flex flex-col gap-3">
      <div className="aspect-video w-full overflow-hidden rounded-lg border bg-black">
        <SplatCanvas flip={flip} key={`${selected.url}:${flip}`} url={selected.url} />
      </div>

      <div className="flex flex-wrap items-center gap-2">
        <label className="text-muted-foreground text-sm" htmlFor="scene">
          Scene
        </label>
        <select
          className="rounded-md border bg-background px-2 py-1 text-sm"
          id="scene"
          onChange={(e) => setSelectedId(e.target.value)}
          value={selected.id}
        >
          {robotSplats.length > 0 ? (
            <optgroup label="Robot captures">
              {scenes
                .filter((s) => s.kind === "robot" && s.id !== "custom")
                .map((s) => (
                  <option key={s.id} value={s.id}>
                    {s.name}
                  </option>
                ))}
            </optgroup>
          ) : null}
          <optgroup label="Demos">
            {DEMO_SCENES.map((s) => (
              <option key={s.id} value={s.id}>
                {s.name}
              </option>
            ))}
          </optgroup>
          {customScene ? <option value="custom">Pasted URL</option> : null}
        </select>

        <Button
          onClick={() => setFlip((f) => !f)}
          size="sm"
          variant={flip ? "default" : "outline"}
        >
          Flip {flip ? "on" : "off"}
        </Button>
      </div>

      <div className="flex items-center gap-2">
        <Input
          className="text-sm"
          onChange={(e) => setUrlInput(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === "Enter") loadUrl();
          }}
          placeholder="Paste a splat URL (.spz / .ply / .splat) to preview"
          value={urlInput}
        />
        <Button onClick={loadUrl} size="sm" variant="outline">
          Load
        </Button>
      </div>

      {listError ? (
        <p className="text-muted-foreground text-xs">
          (couldn&apos;t reach the splat list — showing demos only: {listError})
        </p>
      ) : null}
    </div>
  );
}
