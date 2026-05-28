"use client";

import { useCallback, useEffect, useRef, useState } from "react";

type LiveQuery<T> = {
  data: T | null;
  error: string | null;
  loading: boolean;
  refetch: () => Promise<void>;
};

// One place for the app's "poll an oRPC query on an interval" pattern. Cancels
// in-flight work on unmount, and only swaps state when the payload actually
// changes so live views don't flicker every tick. `refetch` lets children force
// an immediate refresh after a mutation.
export function useLiveQuery<T>(
  fn: () => Promise<T>,
  intervalMs: number,
  deps: unknown[] = [],
): LiveQuery<T> {
  const [data, setData] = useState<T | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);
  const fnRef = useRef(fn);
  fnRef.current = fn;
  const activeRef = useRef(true);

  const load = useCallback(async () => {
    try {
      const next = await fnRef.current();
      if (!activeRef.current) return;
      setData((prev) =>
        JSON.stringify(prev) === JSON.stringify(next) ? prev : next,
      );
      setError(null);
    } catch (e) {
      if (activeRef.current) {
        setError(e instanceof Error ? e.message : "failed to load");
      }
    } finally {
      if (activeRef.current) setLoading(false);
    }
  }, []);

  // biome-ignore lint/correctness/useExhaustiveDependencies: deps are caller-controlled inputs to fn
  useEffect(() => {
    activeRef.current = true;
    setLoading(true);
    load();
    const t = setInterval(load, intervalMs);
    return () => {
      activeRef.current = false;
      clearInterval(t);
    };
  }, [load, intervalMs, ...deps]);

  return { data, error, loading, refetch: load };
}
