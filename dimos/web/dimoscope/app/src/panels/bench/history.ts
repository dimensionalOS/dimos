// Run-history persistence — full-fidelity records stay in React state for the session;
// what hits localStorage is bucket-stripped and trimmed (time-series are the heavy part —
// the JSON export carries them, the disk copy doesn't). The pinned baseline never trims.
import { useCallback, useRef, useState } from "react";
import { reviveRun, type RunRecord, serializeRun, stripBuckets, trimRuns } from "@dimos/web";

const RUNS_KEY = "dimos.bench.history.v1";
const BASE_KEY = "dimos.bench.baseline.v1";
const MAX_RUNS = 20;
const MAX_BYTES = 1_500_000;

function loadRuns(): RunRecord[] {
  try {
    const raw = localStorage.getItem(RUNS_KEY);
    if (!raw) return [];
    const arr = JSON.parse(raw);
    if (!Array.isArray(arr)) return [];
    // Corrupt/unknown-version entries drop individually — one bad record can't nuke the list.
    return arr.map(reviveRun).filter((r): r is RunRecord => !!r);
  } catch {
    return [];
  }
}

function persist(records: RunRecord[], baselineId: string | null): boolean {
  const disk = trimRuns(records.map(stripBuckets), {
    maxRuns: MAX_RUNS,
    maxBytes: MAX_BYTES,
    keepId: baselineId ?? undefined,
  });
  const write = (rs: RunRecord[]) =>
    localStorage.setItem(RUNS_KEY, `[${rs.map(serializeRun).join(",")}]`);
  try {
    write(disk);
    return true;
  } catch {
    try {
      // QuotaExceededError — evict one more, retry once.
      write(disk.slice(0, Math.max(1, disk.length - 1)));
      return true;
    } catch {
      return false;
    }
  }
}

export function useBenchHistory() {
  const [records, setRecords] = useState<RunRecord[]>(loadRuns);
  const [baselineId, setBaselineIdState] = useState<string | null>(
    () => localStorage.getItem(BASE_KEY),
  );
  const [persistOk, setPersistOk] = useState(true);
  // Ref mirror: the runner upserts per-cell between awaits — a useCallback closure over
  // `records` would go stale, and updater-form side effects double-fire under StrictMode.
  const ref = useRef(records);
  const baseRef = useRef(baselineId);

  const commit = useCallback((next: RunRecord[]) => {
    ref.current = next;
    setRecords(next);
    setPersistOk(persist(next, baseRef.current));
  }, []);

  /** Insert or replace by id (the runner re-upserts the in-progress record after every cell,
   *  so a tab crash mid-matrix loses at most one cell). */
  const upsert = useCallback((rec: RunRecord) => {
    commit([rec, ...ref.current.filter((r) => r.id !== rec.id)]);
  }, [commit]);

  const remove = useCallback((id: string) => {
    commit(ref.current.filter((r) => r.id !== id));
  }, [commit]);

  const rename = useCallback((id: string, name: string) => {
    commit(ref.current.map((r) => (r.id === id ? { ...r, name: name || undefined } : r)));
  }, [commit]);

  const pin = useCallback((id: string | null) => {
    baseRef.current = id;
    setBaselineIdState(id);
    if (id) localStorage.setItem(BASE_KEY, id);
    else localStorage.removeItem(BASE_KEY);
  }, []);

  const get = useCallback((id: string) => ref.current.find((r) => r.id === id), []);

  return { records, baselineId, persistOk, upsert, remove, rename, pin, get };
}

export type BenchHistory = ReturnType<typeof useBenchHistory>;
