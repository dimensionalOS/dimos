// BenchTab — interactive, in-app transport benchmark. Unlike the standalone bench.html (which opens
// 3 dedicated clients), this measures the LIVE active transport (whatever the topbar dropdown picked)
// while you vary client-side QoS knobs, with live sparklines + a results table you can copy as Markdown.
//
// Two axes meet here: the workload (canonical STREAM_PROFILES, matching the headless bench/matrix STREAM
// axis so rows line up with RESULTS-mechanisms-<id>.md) × QoS (maxHz, server-vs-client rate limit,
// conflation, + an optional server-json decode A/B via a sibling client). Heavy streams (img/grid) are
// opt-in so opening the tab never floods the active transport.
import { useEffect, useRef, useState } from "react";
import {
  useCaps,
  useCommands,
  useDimosClient,
  useRpc,
  useServers,
  useTopics,
  useTopicStats,
} from "@dimos/react";
import {
  type BenchRow,
  connect,
  createGatewayWsTransport,
  type DimosClient,
  formatMarkdown,
  measureScenario,
  type Qos,
  STREAM_PROFILES,
} from "@dimos/topics";
import { Sparkline } from "../widgets/Sparkline";

const HZ_PRESETS = [0, 5, 20, 60, 120];
const HEAVY = (t: string) =>
  t.includes("/img") || t.includes("/grid") || t.includes("/cloud") ||
  t.includes("/points") || t.includes("/map") || t.includes("/rgb") || t.includes("/depth");

// Large-stream tiers — sustained THROUGHPUT (bytes/frame × rate), grounded in real robot sensor
// bitrates. Frames stay ≤10 MB; high MB/s comes from the rate, not one giant packet. Drives the
// BenchLoad blueprint over RPC (start_bench heavy_hz/heavy_bytes). See docs/benchmarks.md.
interface StreamTier {
  id: string;
  bytes: number;
  hz: number;
  note: string;
}
const STREAM_TIERS: StreamTier[] = [
  { id: "light", bytes: 200_000, hz: 10, note: "2D lidar ~2 MB/s" },
  { id: "camera", bytes: 550_000, hz: 20, note: "1080p ~11 MB/s" },
  { id: "dense", bytes: 1_000_000, hz: 20, note: "depth ~20 MB/s" },
  { id: "depth-hd", bytes: 2_500_000, hz: 20, note: "RealSense/Ouster ~50 MB/s" },
  { id: "raw-1080p", bytes: 6_000_000, hz: 30, note: "raw RGB ~180 MB/s" },
  { id: "firehose", bytes: 10_000_000, hz: 30, note: "raw 4K / multi-cam ~300 MB/s" },
];
const mbps = (bytes: number, hz: number) => (bytes * hz) / 1e6;

function qosLabel(q: Qos): string {
  const hz = q.maxHz ? `${q.maxHz}Hz` : "∞";
  return `maxHz=${hz} · ${q.rateLimit}${q.conflation === "all" ? " · all" : ""}`;
}

// ── live monitor row: keeps its own subscription so stats() is non-zero, applies the live QoS,
//    and feeds a rolling sparkline. Heavy topics are gated by `enabled`. ──────────────────────
function MonitorRow({ topic, qos, enabled }: { topic: string; qos: Qos; enabled: boolean }) {
  const client = useDimosClient();
  const stats = useTopicStats(enabled ? topic : null, 250);
  const [hist, setHist] = useState<{ hz: number; kb: number }[]>([]);
  const qosKey = `${qos.maxHz}|${qos.rateLimit}|${qos.conflation}`;

  // Subscribe to keep bytes flowing (stats() reads a passive rolling window); unsubscribe on disable.
  useEffect(() => {
    if (!client || !enabled) return;
    const t = client.topic(topic);
    const sub = t.subscribe(() => {}); // keep-alive only — the StatsBar/sparkline read t.stats()
    return () => sub.unsubscribe();
  }, [client, topic, enabled]);

  // Re-apply QoS when the knobs change (no resubscribe — setQos re-issues the wire subscribe).
  useEffect(() => {
    if (client && enabled) client.topic(topic).setQos({ ...qos });
  }, [client, topic, enabled, qosKey]);

  useEffect(() => {
    if (!enabled) {
      setHist([]);
      return;
    }
    if (!stats) return;
    setHist((h) => [...h.slice(-59), { hz: stats.hz, kb: stats.bytesPerSec / 1000 }]);
  }, [stats, enabled]);

  const kb = stats ? stats.bytesPerSec / 1000 : 0;
  return (
    <tr style={{ opacity: enabled ? 1 : 0.45 }}>
      <td className="mono">{topic}</td>
      <td>{enabled ? (stats?.hz ?? 0) : "–"}</td>
      <td>{enabled ? kb.toFixed(1) : "–"}</td>
      <td>{enabled && stats?.lastLatencyMs != null ? stats.lastLatencyMs.toFixed(1) : "–"}</td>
      <td>
        <Sparkline data={hist.map((h) => h.kb)} color="var(--accent)" />
      </td>
    </tr>
  );
}

export function BenchTab() {
  const client = useDimosClient();
  const caps = useCaps();
  const commands = useCommands();
  const { call } = useRpc();
  const { servers, activeId } = useServers();
  const discovered = useTopics();
  const activeUrl = servers.find((s) => s.id === activeId)?.url;
  const activeLabel = servers.find((s) => s.id === activeId)?.label ?? "active";

  // ── load generator (BenchLoad RPC): drive the large stream from the browser ──
  const hasRpc = commands.some((c) => c.target === "BenchLoad");
  const [heavyKind, setHeavyKind] = useState<"image" | "cloud">("image");
  const [heavyHz, setHeavyHz] = useState(20);
  const [heavyBytes, setHeavyBytes] = useState(1_000_000);
  const [heavyOn, setHeavyOn] = useState(false);
  const [genMsg, setGenMsg] = useState<string>();
  const [genBusy, setGenBusy] = useState(false);
  const heavyTopic = heavyKind === "cloud" ? "/bench/cloud" : "/bench/img";

  async function applyGen(on: boolean) {
    setGenBusy(true);
    try {
      const res = on
        ? await call<string>("BenchLoad", "start_bench", 100, 20, heavyHz, heavyBytes, heavyKind)
        : await call<string>("BenchLoad", "stop_bench");
      setHeavyOn(on);
      setGenMsg(String(res));
    } catch (e) {
      setGenMsg(`✗ ${(e as Error).message}`);
    } finally {
      setGenBusy(false);
    }
  }

  // QoS knobs
  const [maxHz, setMaxHz] = useState(0);
  const [rateLimit, setRateLimit] = useState<"server" | "client">("server");
  const [conflation, setConflation] = useState<"latest" | "all">("latest");
  const [decodeAb, setDecodeAb] = useState(false);
  const qos: Qos = { maxHz, rateLimit, conflation };

  // Workload selection + which topics are live-monitored (heavy off by default).
  const [picked, setPicked] = useState<string[]>(["pose"]);
  const [showHeavy, setShowHeavy] = useState(false);
  const [dur, setDur] = useState(4000);

  // Run state
  const [running, setRunning] = useState(false);
  const [progress, setProgress] = useState("idle");
  const [rows, setRows] = useState<{ label: string; row: BenchRow }[]>([]);
  const aborted = useRef(false);
  useEffect(() => () => {
    aborted.current = true;
  }, []);

  const profiles = STREAM_PROFILES.filter((p) => picked.includes(p.id));
  // The large stream the generator is currently emitting, added as its own measure scenario.
  const offeredMbps = mbps(heavyBytes, heavyHz);
  const runScenarios = [
    ...profiles.map((p) => ({ name: p.id, topics: p.topics })),
    ...(heavyOn ? [{ name: `heavy:${heavyKind} (≈${offeredMbps.toFixed(0)}MB/s offered)`, topics: [heavyTopic] }] : []),
  ];
  // Topics to live-monitor = union of run scenarios' topics (dedup), heavy ones gated.
  const monitorTopics = [...new Set(runScenarios.flatMap((s) => s.topics))];
  const qosHonored = caps?.qos?.maxHz; // "server" | "client" | undefined
  const serverIgnored = rateLimit === "server" && qosHonored === "client";

  async function run() {
    if (!client) return;
    setRunning(true);
    aborted.current = false;
    setRows([]);
    const out: { label: string; row: BenchRow }[] = [];
    for (const scenario of runScenarios) {
      if (aborted.current) break;
      // 1) the live active transport under the chosen QoS
      setProgress(`${activeLabel} · ${scenario.name} · ${qosLabel(qos)}…`);
      try {
        const r = await measureScenario(client, scenario, dur, true, qos);
        out.push({ label: `${activeLabel} · ${qosLabel(qos)}`, row: { ...r, scenario: scenario.name } });
        setRows([...out]);
      } catch (e) {
        setProgress(`${scenario.name} failed: ${String(e).slice(0, 80)}`);
      }
      // 2) optional decode A/B — a sibling client to the SAME gateway with server-json decode.
      if (decodeAb && activeUrl && !aborted.current) {
        setProgress(`${activeLabel} · ${scenario.name} · server-json decode…`);
        let sib: DimosClient | undefined;
        try {
          sib = await connect({
            transport: createGatewayWsTransport({ url: activeUrl, reconnect: false, decode: "server-json" }),
          });
          await new Promise((r) => setTimeout(r, 250));
          const r = await measureScenario(sib, scenario, dur, true, qos);
          out.push({ label: `${activeLabel} · server-json`, row: { ...r, scenario: scenario.name } });
          setRows([...out]);
        } catch (e) {
          setProgress(`server-json failed: ${String(e).slice(0, 80)}`);
        } finally {
          sib?.close();
        }
      }
    }
    setProgress(aborted.current ? "stopped" : "done ✓");
    setRunning(false);
  }

  // Markdown export — group rows by label and reuse the SDK's formatMarkdown.
  const stamp = new Date().toISOString().slice(0, 16).replace("T", " ");
  const byLabel = new Map<string, BenchRow[]>();
  for (const { label, row } of rows) (byLabel.get(label) ?? byLabel.set(label, []).get(label)!).push(row);
  const md = `# dimoscope in-app bench — ${activeLabel}\n\n_${stamp} · ${dur}ms/scenario · end-to-end latency · QoS: ${qosLabel(qos)}_\n\n` +
    [...byLabel].map(([label, rs]) => formatMarkdown(label, activeUrl ?? activeLabel, dur, stamp, rs)).join("\n---\n\n");

  return (
    <div className="bench-tab">
      <div className="panel-title">
        Bench · live transport ({activeLabel}) · workload × QoS
      </div>

      <div className="bench-grid">
        {/* ── controls ── */}
        <div className="panel bench-controls">
          <div className="bench-section">
            <div className="bench-label">Workload (STREAM profile)</div>
            <div className="bench-chips">
              {STREAM_PROFILES.map((p) => (
                <button
                  key={p.id}
                  className={`tab ${picked.includes(p.id) ? "tab-active" : ""}`}
                  title={p.hint}
                  onClick={() =>
                    setPicked((ps) => ps.includes(p.id) ? ps.filter((x) => x !== p.id) : [...ps, p.id])}
                >
                  {p.id}
                </button>
              ))}
            </div>
            <div className="muted small" style={{ marginTop: 4 }}>
              {profiles.map((p) => `${p.id}: ${p.hint}`).join("  ·  ") || "pick ≥1 workload"}
            </div>
            <div className="muted small" style={{ marginTop: 4 }}>
              The lidar/camera/dense profiles measure <span className="mono">/bench/img</span>. To make
              that topic flow at a chosen size/rate, drive the generator below (or run a headless{" "}
              <span className="mono">bench_source.py</span>).
            </div>
          </div>

          {/* ── load generator: drive the BenchLoad blueprint's large stream over RPC ── */}
          <div className="bench-section">
            <div className="bench-label">Load generator · large stream (BenchLoad RPC)</div>
            {!hasRpc && (
              <div className="muted small">
                No <span className="mono">BenchLoad</span> RPC advertised — run{" "}
                <span className="mono">go2-bench</span> / <span className="mono">bench-load</span> on this
                transport to drive the stream from here.
              </div>
            )}
            <div className="bench-row">
              <span className="muted small">kind</span>
              {(["image", "cloud"] as const).map((k) => (
                <button
                  key={k}
                  className={`tab ${heavyKind === k ? "tab-active" : ""}`}
                  disabled={!hasRpc}
                  onClick={() => setHeavyKind(k)}
                  title={k === "cloud" ? "PointCloud2 (lidar/depth)" : "raw RGB Image (camera frame)"}
                >
                  {k}
                </button>
              ))}
            </div>
            <div className="bench-chips">
              {STREAM_TIERS.map((t) => (
                <button
                  key={t.id}
                  className={`tab ${heavyBytes === t.bytes && heavyHz === t.hz ? "tab-active" : ""}`}
                  disabled={!hasRpc}
                  title={`${t.note} — ${(t.bytes / 1e6).toFixed(2)}MB × ${t.hz}Hz`}
                  onClick={() => {
                    setHeavyBytes(t.bytes);
                    setHeavyHz(t.hz);
                  }}
                >
                  {t.id}
                </button>
              ))}
            </div>
            <div className="bench-row">
              <span className="muted small">frame</span>
              <input
                type="number"
                min={1000}
                step={100_000}
                value={heavyBytes}
                disabled={!hasRpc}
                onChange={(e) => setHeavyBytes(Math.min(Number(e.target.value) || 1000, 12_000_000))}
                style={{ width: 110 }}
              />
              <span className="muted small">B ×</span>
              <input
                type="number"
                min={0}
                step={5}
                value={heavyHz}
                disabled={!hasRpc}
                onChange={(e) => setHeavyHz(Math.max(0, Number(e.target.value) || 0))}
                style={{ width: 64 }}
              />
              <span className="muted small">Hz ⇒ ≈{offeredMbps.toFixed(1)} MB/s offered</span>
            </div>
            <div className="bench-row">
              <button className="tab" disabled={!hasRpc || genBusy} onClick={() => applyGen(true)}>
                {heavyOn ? "Re-apply" : "▶ Apply"}
              </button>
              <button className="tab" disabled={!hasRpc || genBusy || !heavyOn} onClick={() => applyGen(false)}>
                Stop stream
              </button>
              {heavyOn && <span className="badge">{heavyTopic} live</span>}
              {genMsg && <span className="muted small" title={genMsg}>{genMsg.slice(0, 60)}</span>}
            </div>
            <div className="muted small">
              Throughput = frame × rate (frames capped 12 MB; reach high MB/s via rate, not one giant
              packet). Ramp tiers and watch <em>delivered</em> kB/s plateau below <em>offered</em> — that
              gap is the pipeline ceiling.
            </div>
          </div>

          <div className="bench-section">
            <div className="bench-label">QoS · client-side</div>
            <div className="bench-row">
              <span className="muted small">maxHz</span>
              {HZ_PRESETS.map((h) => (
                <button
                  key={h}
                  className={`tab ${maxHz === h ? "tab-active" : ""}`}
                  onClick={() => setMaxHz(h)}
                >
                  {h === 0 ? "∞" : h}
                </button>
              ))}
            </div>
            <div className="bench-row">
              <span className="muted small">rate limit</span>
              {(["server", "client"] as const).map((r) => (
                <button
                  key={r}
                  className={`tab ${rateLimit === r ? "tab-active" : ""}`}
                  onClick={() => setRateLimit(r)}
                >
                  {r}
                </button>
              ))}
              <span className={`badge ${serverIgnored ? "badge-warn" : ""}`} title="from the active transport's caps.qos">
                {qosHonored ? `caps: ${qosHonored}-side` : "caps: client only"}
              </span>
            </div>
            {serverIgnored && (
              <div className="muted small">
                ⚠ this transport has no server downsample — server request is ignored; only client throttling applies.
              </div>
            )}
            <div className="bench-row">
              <span className="muted small">conflation</span>
              {(["latest", "all"] as const).map((c) => (
                <button
                  key={c}
                  className={`tab ${conflation === c ? "tab-active" : ""}`}
                  onClick={() => setConflation(c)}
                >
                  {c}
                </button>
              ))}
              <span className="muted small">derived from maxHz — the SDK never queues</span>
            </div>
            <label className="bench-row muted small">
              <input
                type="checkbox"
                checked={decodeAb}
                disabled={!activeUrl}
                onChange={(e) => setDecodeAb(e.target.checked)}
              />
              also measure server-json decode (sibling client) {activeUrl ? "" : "— gateway transports only"}
            </label>
          </div>

          <div className="bench-section">
            <div className="bench-label">QoS · gateway scheduler (auto, per-lane)</div>
            <div className="muted small">
              priority/reliability/depth are wired end-to-end — the gateway-WS adapter forwards them and
              the server's per-client priority outbox honors them (assigned automatically per lane; see
              the QoS demo, not toggles here). Per-provider knobs (zenoh congestion/express, WebRTC
              ordered/maxRetransmits — set at channel creation) aren't modeled yet.
            </div>
            <div className="bench-row" style={{ opacity: 0.5 }}>
              {["priority", "reliability", "depth"].map((k) => (
                <button key={k} className="tab" disabled title="auto per lane (gateway scheduler)">{k}</button>
              ))}
            </div>
          </div>

          <div className="bench-section">
            <div className="bench-row">
              <span className="muted small">duration</span>
              <input
                type="number"
                min={1000}
                step={1000}
                value={dur}
                onChange={(e) => setDur(Number(e.target.value) || 4000)}
                style={{ width: 80 }}
              />
              <span className="muted small">ms</span>
            </div>
            <div className="bench-row">
              <button className="tab" onClick={run} disabled={running || !client || !runScenarios.length}>
                {running ? "running…" : "▶ Run sweep"}
              </button>
              <button className="tab" disabled={!running} onClick={() => (aborted.current = true)}>
                Stop
              </button>
              <button className="tab" disabled={!rows.length} onClick={() => navigator.clipboard.writeText(md)}>
                copy Markdown
              </button>
              <span className="muted small">{progress}</span>
            </div>
          </div>
        </div>

        {/* ── live monitor ── */}
        <div className="panel">
          <div className="panel-title">Live · {activeLabel}</div>
          <label className="muted small bench-row">
            <input type="checkbox" checked={showHeavy} onChange={(e) => setShowHeavy(e.target.checked)} />
            monitor heavy streams (/bench/img, /bench/cloud, /bench/grid) — off by default (can be ≫10 MB/s)
          </label>
          <table className="stats" style={{ width: "100%", marginTop: 8 }}>
            <thead>
              <tr><th>topic</th><th>hz</th><th>kB/s</th><th>lat ms</th><th>kB/s trend</th></tr>
            </thead>
            <tbody>
              {monitorTopics.length === 0 && (
                <tr><td colSpan={5} className="muted small">pick a workload to monitor</td></tr>
              )}
              {monitorTopics.map((t) => (
                <MonitorRow key={t} topic={t} qos={qos} enabled={!HEAVY(t) || showHeavy} />
              ))}
            </tbody>
          </table>
          <div className="muted small" style={{ marginTop: 6 }}>
            discovered /bench/* on this transport:{" "}
            {discovered.filter((t) => t.topic.startsWith("/bench/")).map((t) => t.topic).join(", ") || "none yet"}
          </div>
        </div>
      </div>

      {/* ── results ── */}
      {rows.length > 0 && (
        <div className="panel" style={{ marginTop: 12 }}>
          <div className="panel-title">Results · end-to-end (publish→browser)</div>
          <table className="stats" style={{ width: "100%" }}>
            <thead>
              <tr>
                <th>config</th><th>scenario</th><th>hz</th><th>kB/s</th>
                <th>p50</th><th>p95</th><th>p99</th><th>loss%</th>
              </tr>
            </thead>
            <tbody>
              {rows.map(({ label, row }, i) => (
                <tr key={i}>
                  <td className="mono">{label}</td>
                  <td className="mono">{row.scenario}</td>
                  <td>{row.hz}</td>
                  <td>{row.kbps}</td>
                  <td>{row.latP50}</td>
                  <td>{row.latP95}</td>
                  <td>{row.latP99}</td>
                  <td>{row.lossPct}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
    </div>
  );
}
