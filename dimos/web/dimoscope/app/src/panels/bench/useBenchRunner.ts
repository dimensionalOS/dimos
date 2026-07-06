// The sweep executor — walks a SweepPlan cell by cell: asserts netem per group (settle,
// verify from the POST body, time-based re-assert against the self-heal), drives the GO2Load
// generator per workload (on config change only), measures, stamps each cell with the
// condition that actually held (netem · maxHz · wire), and restores state on every exit path
// (generator first, then netem — the restore POST must not crawl through a saturated link).
import { useCallback, useEffect, useRef, useState } from "react";
import {
  type BenchBucket,
  type BenchRow,
  type ClockSample,
  defaultGrace,
  defaultWarmup,
  type GeneratorConfig,
  measureScenario,
  type RunCell,
  type RunMeta,
} from "@dimos/web";
import { useDimosClient, useRpc } from "../../dimos";
import { useNetem } from "../../netem";
import type { BenchHistory } from "./history";
import { type GenWant, newRunId, type SweepPlan, synthRecord } from "./model";

const NETEM_SETTLE_MS = 1000;
const GEN_SETTLE_MS = 300;
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

const emptyRow = (scenario: string, topics: number): BenchRow => ({
  scenario,
  topics,
  msgs: 0,
  hz: 0,
  kbps: 0,
  latP50: NaN,
  latP95: NaN,
  latP99: NaN,
  latMax: NaN,
  latStd: NaN,
  lossPct: NaN,
  latePct: NaN,
  elapsedMs: 0,
  offeredHz: NaN,
  offeredKbps: NaN,
  deliveryPct: NaN,
  seqResets: 0,
  lanes: [],
  buckets: [],
});

export interface SweepCtx {
  durMs: number;
  /** UI transport selection at start (cells stamp it; the wire may differ). */
  transportLabel: string;
  gateway: string;
  reproUrl: string;
  /** Manual flood flowing at sweep start — the restore target when the sweep drives the generator. */
  manualGen: GeneratorConfig | null;
}

export function useBenchRunner(history: BenchHistory) {
  const client = useDimosClient();
  const { call } = useRpc();
  const netem = useNetem();

  const [cells, setCells] = useState<RunCell[]>([]);
  const [loadedIds, setLoadedIds] = useState<Set<string>>(new Set());
  // id → the exact cell array appended; past history records are immutable, so cell object
  // identity is stable and lets us remove exactly that run's cells on toggle-off.
  const loadedCells = useRef<Map<string, RunCell[]>>(new Map());
  const [running, setRunning] = useState(false);
  const [progress, setProgress] = useState("idle");
  const [note, setNote] = useState<string>();
  const [live, setLive] = useState<BenchBucket[]>([]);
  const [clock, setClock] = useState<ClockSample>();

  const abortedRef = useRef(false);
  const runningRef = useRef(false);
  // A transport/gateway switch rebuilds the client — the captured one is a corpse; abort
  // rather than measure it. Unmount aborts the same way.
  const clientRef = useRef(client);
  useEffect(() => {
    if (runningRef.current && clientRef.current && client !== clientRef.current) {
      abortedRef.current = true;
    }
    clientRef.current = client;
  }, [client]);
  useEffect(() => () => {
    abortedRef.current = true;
  }, []);

  const stop = useCallback(() => {
    if (!runningRef.current) return;
    abortedRef.current = true;
    setProgress("stopping after this cell…");
  }, []);

  const clear = useCallback(() => {
    loadedCells.current.clear();
    setLoadedIds(new Set());
    setCells([]);
  }, []);
  // Toggle a past run's cells into/out of the table (reversible; other loaded runs untouched).
  const toggleLoad = useCallback((rec: { id: string; cells: RunCell[] }) => {
    setLoadedIds((prev) => {
      const next = new Set(prev);
      if (next.has(rec.id)) {
        const set = new Set(loadedCells.current.get(rec.id));
        loadedCells.current.delete(rec.id);
        setCells((cs) => cs.filter((c) => !set.has(c)));
        next.delete(rec.id);
      } else {
        // Append only cells not already in the table: a live/just-finished run shares cell object
        // identity with its history record, and re-appending it duplicated every row in the
        // copied Markdown. Track exactly what we appended so toggle-off removes only that.
        setCells((cs) => {
          const present = new Set(cs);
          const fresh = rec.cells.filter((c) => !present.has(c));
          loadedCells.current.set(rec.id, fresh);
          return [...cs, ...fresh];
        });
        next.add(rec.id);
      }
      return next;
    });
  }, []);

  const start = useCallback(async (plan: SweepPlan, ctx: SweepCtx) => {
    if (!client || runningRef.current || plan.cells.length === 0) return;
    runningRef.current = true;
    setRunning(true);
    abortedRef.current = false;
    setNote(undefined);
    const runId = newRunId();
    const stamp = new Date().toISOString().slice(0, 16).replace("T", " ");

    // Clock probe ONCE, pre-matrix: netem shapes egress only, so probing under a delay
    // profile would bias the NTP midpoint by ~delay/2; skew doesn't change per profile.
    setProgress("clock sync…");
    const off = await client.estimateClockOffset().catch(() => undefined);
    setClock(off);
    // Refresh netem at start — a profile may have self-healed since the panel last looked.
    const net0 = await netem.refresh();
    const preNetem = net0?.enabled ? net0.active : null;
    const baseNet = net0?.enabled ? net0.active : "live";
    const healS = net0?.healsInS ?? 900;

    const meta: RunMeta = {
      transportLabel: ctx.transportLabel,
      gatewayUrl: ctx.gateway,
      origin: globalThis.location?.origin,
      userAgent: globalThis.navigator?.userAgent,
      durMs: ctx.durMs,
      warmupMs: defaultWarmup(ctx.durMs),
      graceMs: defaultGrace(ctx.durMs),
      clock: off,
      reproUrl: ctx.reproUrl,
    };

    const runCells: RunCell[] = [];
    let curNet: string | null = null;
    let lastAssert = 0;
    let touchedNetem = false;
    // Best knowledge of what's flowing — a GUESS, seeded from this browser's manual state.
    // Another client may be flooding, and a timed-out start_bench may have applied server-side
    // anyway; at the trust boundaries (first off, restore) we send regardless — it's idempotent.
    let genCur: GeneratorConfig | "off" = ctx.manualGen ?? "off";
    let offConfirmed = false;

    const ensureGen = async (want: GenWant): Promise<GeneratorConfig | null> => {
      if (want.kind === "keep") return genCur === "off" ? null : genCur;
      try {
        if (want.kind === "off") {
          if (!offConfirmed) {
            await call("GO2Load", "stop_bench");
            genCur = "off";
            offConfirmed = true;
            await sleep(GEN_SETTLE_MS);
          }
          return null;
        }
        const spec: GeneratorConfig = {
          hz: want.spec.hz,
          bytes: want.spec.bytes,
          kind: want.spec.kind,
        };
        const same = genCur !== "off" && genCur.hz === spec.hz && genCur.bytes === spec.bytes &&
          genCur.kind === spec.kind;
        if (!same) {
          offConfirmed = false; // even a throwing start may have applied server-side
          await call("GO2Load", "start_bench", spec.hz, spec.bytes, spec.kind);
          genCur = spec;
          await sleep(GEN_SETTLE_MS);
        }
        return spec;
      } catch (e) {
        setNote(`generator ✗ ${String((e as Error).message ?? e).slice(0, 60)} — measuring whatever flows`);
        return null;
      }
    };

    const assertNetem = async (id: string): Promise<string> => {
      const st = await netem.apply(id); // shared context — the topbar select tracks each cell
      if (st?.active === id) {
        touchedNetem = true;
        curNet = id;
        lastAssert = Date.now();
        await sleep(NETEM_SETTLE_MS);
        return id;
      }
      setNote(`netem ${id} ✗ — running on the actual path instead`);
      return st?.active ?? baseNet;
    };

    try {
      for (let i = 0; i < plan.cells.length; i++) {
        if (abortedRef.current) break;
        const pc = plan.cells[i];
        let cellNet = baseNet;
        if (pc.netem !== null) {
          // Assert on group change; re-assert on long matrices before the profile self-heals.
          // Floor healS: a misconfigured HEAL_S=0 must not force a re-assert on every cell.
          const stale = Date.now() - lastAssert > Math.min(Math.max(healS, 60) / 2, 300) * 1000;
          cellNet = pc.netem !== curNet || stale ? await assertNetem(pc.netem) : pc.netem;
        }
        if (abortedRef.current) break;
        const gen = await ensureGen(pc.want);
        if (abortedRef.current) break;

        setProgress(
          `cell ${i + 1}/${plan.cells.length} · ${cellNet} · ${pc.profile.id} · maxHz ${
            pc.maxHz ? `${pc.maxHz}Hz` : "∞"
          }${plan.axes.reps > 1 ? ` · rep ${pc.rep}` : ""}`,
        );
        setLive([]);
        const wireBefore = client.gatewayLabel;
        let cell: RunCell;
        try {
          const row = await measureScenario(
            client,
            { name: pc.profile.id, topics: pc.profile.topics },
            ctx.durMs,
            true,
            { maxHz: pc.maxHz },
            { offsetMs: off?.offsetMs, onBucket: (b) => setLive((l) => [...l, b]) },
          );
          const wireAfter = client.gatewayLabel;
          const wire = wireBefore && wireAfter && wireBefore !== wireAfter
            ? `${wireBefore}→${wireAfter}` // mid-cell failover, stamped honestly
            : wireAfter ?? wireBefore;
          cell = {
            scenario: pc.profile.id,
            netem: cellNet,
            maxHz: pc.maxHz,
            transport: ctx.transportLabel,
            wire,
            rep: pc.rep,
            gen,
            row,
          };
        } catch (e) {
          // A dead cell is data (the failover/timeout IS the finding) — record and continue.
          cell = {
            scenario: pc.profile.id,
            netem: cellNet,
            maxHz: pc.maxHz,
            transport: ctx.transportLabel,
            wire: client.gatewayLabel ?? wireBefore,
            rep: pc.rep,
            gen,
            error: String((e as Error).message ?? e).slice(0, 120),
            row: emptyRow(pc.profile.id, pc.profile.topics.length),
          };
        }
        runCells.push(cell);
        setCells((prev) => [...prev, cell]);
        // Upsert after EVERY cell — a firehose-tier tab kill loses at most one cell.
        history.upsert(synthRecord([...runCells], meta, { id: runId, stamp }));
      }
    } finally {
      setProgress(abortedRef.current ? "stopped · restoring…" : "restoring…");
      // Generator first — cut the flood so the netem-restore POST isn't competing with it.
      if (plan.driveGen) {
        try {
          offConfirmed = false; // restore must not trust the guess — send regardless
          await ensureGen(
            ctx.manualGen
              ? {
                kind: "set",
                spec: {
                  hz: ctx.manualGen.hz,
                  bytes: ctx.manualGen.bytes,
                  kind: ctx.manualGen.kind === "cloud" ? "cloud" : "image",
                },
              }
              : { kind: "off" },
          );
        } catch { /* client may already be gone */ }
      }
      if (touchedNetem && preNetem) {
        try {
          await netem.apply(preNetem);
        } catch { /* self-heal is the backstop */ }
      }
      if (runCells.length) {
        history.upsert(
          synthRecord(runCells, meta, { id: runId, stamp, aborted: abortedRef.current || undefined }),
        );
        // The finished run's cells ARE in the table — mark it loaded so its history button reads
        // "loaded" (toggling it off works) instead of inviting a "load" that duplicates every row.
        loadedCells.current.set(runId, [...runCells]);
        setLoadedIds((prev) => new Set(prev).add(runId));
      }
      runningRef.current = false;
      setRunning(false);
      setLive([]);
      setProgress(abortedRef.current ? "stopped · restored" : "done ✓");
    }
  }, [client, call, netem, history]);

  return { cells, clear, toggleLoad, loadedIds, running, progress, note, live, clock, start, stop };
}

export type BenchRunner = ReturnType<typeof useBenchRunner>;
