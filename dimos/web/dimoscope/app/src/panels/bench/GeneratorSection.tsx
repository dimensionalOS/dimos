// Load-generator section — the GO2Load flood config. The tier ladder renders from the
// gen-bearing STREAM_PROFILES (single source of truth for hz×bytes), free inputs cover
// everything between the rungs, and the auto-drive toggle hands the config to the sweep.
import { useState } from "react";
import { type GeneratorConfig, STREAM_PROFILES } from "@dimos/web";
import { useRpc } from "../../dimos";
import type { BenchUrlConfig } from "./urlParams";

const mbps = (bytes: number, hz: number) => (bytes * hz) / 1e6;

export function GeneratorSection({ hasRpc, cfg, patch, running, manualGen, onManualGen }: {
  hasRpc: boolean;
  cfg: BenchUrlConfig;
  patch: (p: Partial<BenchUrlConfig>) => void;
  running: boolean;
  manualGen: GeneratorConfig | null;
  onManualGen: (g: GeneratorConfig | null) => void;
}) {
  const { call } = useRpc();
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState<string>();
  const heavyTopic = cfg.genKind === "cloud" ? "/load/cloud" : "/load/img";
  const offered = mbps(cfg.genBytes, cfg.genHz);
  const tiers = STREAM_PROFILES.filter((p) => p.gen && p.id !== "mixed");

  async function applyManual(on: boolean) {
    setBusy(true);
    try {
      const res = on
        ? await call<string>("GO2Load", "start_bench", cfg.genHz, cfg.genBytes, cfg.genKind)
        : await call<string>("GO2Load", "stop_bench");
      onManualGen(on ? { kind: cfg.genKind, hz: cfg.genHz, bytes: cfg.genBytes } : null);
      setMsg(String(res));
    } catch (e) {
      setMsg(`✗ ${(e as Error).message}`);
    } finally {
      setBusy(false);
    }
  }

  return (
    <div className="bench-section">
      <div className="bench-label">Load generator · large stream (GO2Load RPC)</div>
      {!hasRpc
        ? (
          <div className="muted small">
            No <span className="mono">GO2Load</span> RPC advertised — run{" "}
            <span className="mono">deno task dog</span> (or <span className="mono">load</span>), to make{" "}
            <span className="mono">/load/*</span> flow. Generator not driven — the sweep measures whatever
            flows (offered still derived from seq spans).
          </div>
        )
        : (
          <>
            <div className="bench-row">
              <span className="muted small">kind</span>
              {(["image", "cloud"] as const).map((k) => (
                <button
                  key={k}
                  className={`tab ${cfg.genKind === k ? "tab-active" : ""}`}
                  onClick={() => patch({ genKind: k })}
                  disabled={running}
                  title={k === "cloud" ? "PointCloud2 (lidar/depth)" : "raw RGB Image (camera frame)"}
                >
                  {k}
                </button>
              ))}
            </div>
            <div className="bench-chips">
              {tiers.map((p) => (
                <button
                  key={p.id}
                  className={`tab ${cfg.genBytes === p.gen!.bytes && cfg.genHz === p.gen!.hz ? "tab-active" : ""}`}
                  title={p.hint}
                  disabled={running}
                  onClick={() => patch({ genHz: p.gen!.hz, genBytes: p.gen!.bytes })}
                >
                  {p.id}
                </button>
              ))}
            </div>
            <div className="bench-row">
              <span className="muted small">hz</span>
              <input
                type="number"
                min={1}
                max={240}
                value={cfg.genHz}
                disabled={running}
                onChange={(e) => patch({ genHz: Math.max(1, Number(e.target.value) || 1) })}
                style={{ width: 56 }}
              />
              <span className="muted small">× bytes</span>
              <input
                type="number"
                min={1000}
                step={100_000}
                value={cfg.genBytes}
                disabled={running}
                onChange={(e) => patch({ genBytes: Math.max(1000, Number(e.target.value) || 1000) })}
                style={{ width: 96 }}
              />
              <span className="muted small">≈ {offered.toFixed(1)} MB/s offered</span>
            </div>
            <div className="bench-row">
              <button
                className={`tab ${cfg.autoDrive ? "tab-active" : ""}`}
                disabled={running}
                title="the sweep starts/stops the flood per heavy profile (its hz×bytes), and restores this manual state after"
                onClick={() => patch({ autoDrive: !cfg.autoDrive })}
              >
                sweep drives generator
              </button>
              <button className="tab" disabled={busy || running} onClick={() => applyManual(true)}>
                {manualGen ? "Re-apply" : "▶ Start load"}
              </button>
              <button
                className="tab"
                disabled={busy || running || !manualGen}
                onClick={() => applyManual(false)}
              >
                Stop load
              </button>
              {manualGen && <span className="badge">{heavyTopic} live</span>}
              {msg && <span className="muted small" title={msg}>{msg.slice(0, 60)}</span>}
            </div>
            {!cfg.autoDrive && (
              <div className="muted small">
                manual only — heavy rows measure whatever flows; the profile name is intent, not config
              </div>
            )}
          </>
        )}
    </div>
  );
}
