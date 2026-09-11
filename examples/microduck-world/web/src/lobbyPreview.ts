import { useEffect, useMemo, useState } from "react";
import type { Session } from "@dimos/sdk";
import {
  fetchDefinition,
  readSnapshot,
  type WorldDefinition,
  type WorldSnapshot,
} from "./worldModel.ts";
import type { DuckPreview } from "./hoverPreview.tsx";

export function useLobbyPreview(session: Session | null) {
  const [model, setModel] = useState<WorldDefinition | null>(null);
  const [snapshot, setSnapshot] = useState<WorldSnapshot | null>(null);
  const [clip, setClip] = useState<
    Omit<DuckPreview, "model"> & { bodyIds: number[] } | null
  >(null);
  useEffect(() => {
    let disposed = false;
    fetch(`${import.meta.env.BASE_URL}duck-preview.json`).then((r) => {
      if (!r.ok) throw new Error("Preview unavailable");
      return r.json();
    }).then((value) => {
      if (!disposed) setClip(value);
    }).catch(() => {});
    return () => {
      disposed = true;
    };
  }, []);
  useEffect(() => {
    let disposed = false;
    let loading = "";
    const read = async () => {
      let value = session?.store.get("world_state")?.value;
      if (!session) {
        try {
          const response = await fetch("/api/preview", { signal: AbortSignal.timeout(5000) });
          if (!response.ok || disposed) return;
          value = await response.json();
        } catch { return; }
      }
      if (disposed) return;
      const next = readSnapshot(value);
      if (!next) return;
      setSnapshot(next);
      if (loading === next.model) return;
      loading = next.model;
      fetchDefinition(next.model).then((definition) => {
        if (disposed) return;
        setModel(definition);
      }).catch(() => {
        loading = "";
      });
    };
    read();
    const timer = setInterval(read, session ? 1000 : 5000);
    return () => {
      disposed = true;
      clearInterval(timer);
    };
  }, [session]);
  const duck = useMemo(() =>
    !clip || !model ? null : ({
      ...clip,
      model: {
        ...model,
        bodyIds: clip.bodyIds,
        geoms: model.geoms.filter((g) => clip.bodyIds.includes(g.body)),
      },
    }), [clip, model]);
  return { model, snapshot, duck };
}
