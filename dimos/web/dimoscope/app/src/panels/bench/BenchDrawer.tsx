// BenchDrawer — the quantitative benchmark, folded into the Topics tab as a collapsed drawer.
// Measures the live transport across a matrix of netem × workload × maxHz (× repeats), one
// RunRecord per sweep: per-cell condition stamps, per-lane stats, time-series buckets,
// localStorage history with a pinned Δ baseline, Markdown/JSON export, and a repro URL.
//
// Config drives from the URL (bench/urlParams.ts registry) and reflects back with defaults
// omitted; ?run=1 auto-runs behind a 3 s cancel chip. Execution lives in useBenchRunner,
// persistence in history.ts — this file owns config state and composition only.
import { useEffect, useMemo, useRef, useState } from "react";
import { defaultGrace, defaultWarmup, type GeneratorConfig, STREAM_PROFILES } from "@dimos/web";
import { useCommands, useDimosClient, useServers, useTopics } from "../../dimos";
import { useGateway } from "../../gateway";
import { useNetem } from "../../netem";
import {
  type BenchUrlConfig,
  buildReproUrl,
  clearRunFlag,
  hasBenchParams,
  readBenchConfig,
  readRunFlag,
  reflectBenchConfig,
} from "./urlParams";
import { buildBaselineIndex, buildPlan, synthRecord } from "./model";
import { useBenchHistory } from "./history";
import { useBenchRunner } from "./useBenchRunner";
import { GeneratorSection } from "./GeneratorSection";
import { NetemSection } from "./NetemSection";
import { RunControls } from "./RunControls";
import { ResultsTable } from "./ResultsTable";
import { HistoryPanel } from "./HistoryPanel";

export { hasBenchParams };

export function BenchDrawer() {
  const client = useDimosClient();
  const commands = useCommands();
  const { servers, activeId } = useServers();
  const discovered = useTopics();
  const { gateway } = useGateway();
  const netem = useNetem();
  const activeLabel = servers.find((s) => s.id === activeId)?.label ?? "active";
  const hasRpc = commands.some((c) => c.target === "GO2Load");

  const [open, setOpen] = useState(hasBenchParams);
  const [cfg, setCfg] = useState<BenchUrlConfig>(readBenchConfig);
  const patch = (p: Partial<BenchUrlConfig>) => setCfg((c) => ({ ...c, ...p }));
  useEffect(() => reflectBenchConfig(cfg), [cfg]);
  const [manualGen, setManualGen] = useState<GeneratorConfig | null>(null);

  const history = useBenchHistory();
  const runner = useBenchRunner(history);

  const plan = useMemo(
    () => buildPlan(cfg, netem.netem, hasRpc),
    [cfg, netem.netem, hasRpc],
  );
  const repro = buildReproUrl(cfg, { gw: gateway, transport: activeId ?? "auto" });

  const runSweep = () =>
    runner.start(plan, {
      durMs: cfg.durMs,
      transportLabel: activeLabel,
      gateway,
      reproUrl: repro,
      manualGen,
    });

  // ?run=1 — one-shot auto-run once the client is up and netem state is known, behind a
  // cancel window (a pasted link flipping gateway-wide netem deserves a beat of consent).
  const [autoRunIn, setAutoRunIn] = useState<number | null>(null);
  const armedRef = useRef(readRunFlag());
  useEffect(() => {
    if (!armedRef.current || autoRunIn !== null || !client || !netem.ready) return;
    setAutoRunIn(3);
  }, [client, netem.ready, autoRunIn]);
  useEffect(() => {
    if (autoRunIn === null) return;
    if (autoRunIn <= 0) {
      armedRef.current = false;
      clearRunFlag();
      setAutoRunIn(null);
      runSweep();
      return;
    }
    const t = setTimeout(() => setAutoRunIn((n) => (n ?? 1) - 1), 1000);
    return () => clearTimeout(t);
  }, [autoRunIn]);
  const cancelAutoRun = () => {
    armedRef.current = false;
    clearRunFlag();
    setAutoRunIn(null);
  };

  const picked = STREAM_PROFILES.filter((p) => cfg.profiles.includes(p.id));
  const benchFlowing = discovered.some((t) => t.topic.startsWith("/load/"));
  const baseline = useMemo(() => {
    const rec = history.baselineId ? history.get(history.baselineId) : undefined;
    return rec ? buildBaselineIndex(rec) : undefined;
    // records identity changes on every upsert; baselineId alone misses renames of the pinned run
  }, [history.baselineId, history.records]);

  const buildRecord = () =>
    synthRecord(runner.cells, {
      transportLabel: activeLabel,
      gatewayUrl: gateway,
      origin: globalThis.location?.origin,
      userAgent: globalThis.navigator?.userAgent,
      durMs: cfg.durMs,
      warmupMs: defaultWarmup(cfg.durMs),
      graceMs: defaultGrace(cfg.durMs),
      clock: runner.clock,
      reproUrl: repro,
    });

  return (
    <div className="panel bench-drawer">
      <button type="button" className="bench-drawer-head" onClick={() => setOpen((o) => !o)}>
        <span className="bench-drawer-caret">{open ? "▾" : "▸"}</span>
        Benchmark
        <span className="muted small">
          — matrix sweep on the live transport ({activeLabel}) → Markdown / JSON · repro URLs
        </span>
      </button>

      {open && (
        <div className="bench-drawer-body">
          {autoRunIn !== null && (
            <div className="bench-row">
              <span className="badge badge-warn">auto-run in {autoRunIn}s</span>
              <button className="tab" onClick={cancelAutoRun}>cancel</button>
              <span className="muted small mono" style={{ overflowWrap: "anywhere" }}>
                {plan.cells.length} cells from the URL
              </span>
            </div>
          )}

          <div className="bench-section">
            <div className="bench-label">Workload (STREAM profile)</div>
            <div className="bench-chips">
              {STREAM_PROFILES.map((p) => (
                <button
                  key={p.id}
                  className={`tab ${cfg.profiles.includes(p.id) ? "tab-active" : ""}`}
                  title={p.hint}
                  disabled={runner.running}
                  onClick={() =>
                    patch({
                      profiles: cfg.profiles.includes(p.id)
                        ? cfg.profiles.filter((x) => x !== p.id)
                        : [...cfg.profiles, p.id],
                    })}
                >
                  {p.id}
                </button>
              ))}
            </div>
            <div className="muted small" style={{ marginTop: 4 }}>
              {picked.map((p) => `${p.id}: ${p.hint}`).join("  ·  ") || "pick ≥1 workload"}
            </div>
          </div>

          <GeneratorSection
            hasRpc={hasRpc}
            cfg={cfg}
            patch={patch}
            running={runner.running}
            manualGen={manualGen}
            onManualGen={setManualGen}
          />

          <NetemSection
            netSel={cfg.net}
            onNetSel={(net) => patch({ net })}
            netIgnored={plan.netIgnored}
            running={runner.running}
          />

          <RunControls
            cfg={cfg}
            patch={patch}
            plan={plan}
            running={runner.running}
            canRun={!!client && plan.cells.length > 0}
            progress={runner.progress}
            note={runner.note}
            live={runner.live}
            onRun={runSweep}
            onStop={runner.stop}
          />
          {!benchFlowing && !manualGen && (
            <div className="bench-section muted small">
              No <span className="mono">/load/*</span> flowing — Start load above, or run{" "}
              <span className="mono">deno task dog</span>, so the sweep has data to measure.
            </div>
          )}

          <ResultsTable
            cells={runner.cells}
            clock={runner.clock}
            baseline={baseline}
            buildRecord={buildRecord}
            onClear={runner.clear}
          />

          <HistoryPanel history={history} onLoad={runner.append} />
        </div>
      )}
    </div>
  );
}
