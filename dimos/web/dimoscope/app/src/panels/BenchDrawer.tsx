// BenchDrawer — the quantitative benchmark, folded into the Topics tab as a collapsed drawer. Measures
// the live active transport across the STREAM_PROFILES workloads → a results table you can copy as Markdown.
//
// The optional Start/Stop control drives the GO2Load blueprint's large stream over its already-whitelisted
// @rpc, so a sweep can generate its own flow without re-implementing the load-gen backend.
import { useEffect, useRef, useState } from "react";
import { useCommands, useDimosClient, useRpc, useServers, useTopics } from "../dimos";
import { useGateway } from "../gateway";
import { type BenchRow, formatMarkdown, measureScenario, type Qos, STREAM_PROFILES } from "@dimos/web";

const HZ_PRESETS = [0, 5, 20, 60, 120];

// Server-side network conditions (GET/POST /netem — gateway/netem.py, opt-in via NETEM_CTL=1 on a
// Linux gateway). The section renders only when the gateway reports it enabled.
interface NetemChip {
  id: string;
  label: string;
  desc: string;
}
interface NetemState {
  enabled: boolean;
  active: string;
  healsInS: number;
  profiles: NetemChip[];
  outages: NetemChip[];
}

// Large-stream tiers grounded in real sensor bitrates; frames stay ≤12 MB, high MB/s comes from rate, not a giant packet.
const STREAM_TIERS = [
  { id: "light", bytes: 200_000, hz: 10, note: "2D lidar ~2 MB/s" },
  { id: "camera", bytes: 550_000, hz: 20, note: "1080p ~11 MB/s" },
  { id: "dense", bytes: 1_000_000, hz: 20, note: "depth ~20 MB/s" },
  { id: "depth-hd", bytes: 2_500_000, hz: 20, note: "RealSense/Ouster ~50 MB/s" },
  { id: "raw-1080p", bytes: 6_000_000, hz: 30, note: "raw RGB ~180 MB/s" },
  { id: "firehose", bytes: 10_000_000, hz: 30, note: "raw 4K / multi-cam ~300 MB/s" },
];
const mbps = (bytes: number, hz: number) => (bytes * hz) / 1e6;

export function BenchDrawer() {
  const client = useDimosClient();
  const commands = useCommands();
  const { call } = useRpc();
  const { servers, activeId } = useServers();
  const discovered = useTopics();
  const activeUrl = servers.find((s) => s.id === activeId)?.url;
  const activeLabel = servers.find((s) => s.id === activeId)?.label ?? "active";

  const [open, setOpen] = useState(false);

  const hasRpc = commands.some((c) => c.target === "GO2Load");
  const [heavyKind, setHeavyKind] = useState<"image" | "cloud">("image");
  const [heavyHz, setHeavyHz] = useState(20);
  const [heavyBytes, setHeavyBytes] = useState(1_000_000);
  const [heavyOn, setHeavyOn] = useState(false);
  const [genMsg, setGenMsg] = useState<string>();
  const [genBusy, setGenBusy] = useState(false);
  const heavyTopic = heavyKind === "cloud" ? "/load/cloud" : "/load/img";
  const offeredMbps = mbps(heavyBytes, heavyHz);

  async function applyGen(on: boolean) {
    setGenBusy(true);
    try {
      const res = on
        ? await call<string>("GO2Load", "start_bench", heavyHz, heavyBytes, heavyKind)
        : await call<string>("GO2Load", "stop_bench");
      setHeavyOn(on);
      setGenMsg(String(res));
    } catch (e) {
      setGenMsg(`✗ ${(e as Error).message}`);
    } finally {
      setGenBusy(false);
    }
  }

  // Network conditions (server-side netem via the gateway; hidden unless the gateway enables it).
  const { gateway } = useGateway();
  const httpBase = `${location.protocol}//${gateway}`;
  const [netem, setNetem] = useState<NetemState | null>(null);
  const [netBusy, setNetBusy] = useState(false);
  const [netMsg, setNetMsg] = useState<string>();

  async function fetchNetem(): Promise<NetemState | null> {
    try {
      const st: NetemState = await (await fetch(`${httpBase}/netem`)).json();
      setNetem(st);
      return st;
    } catch {
      setNetem(null); // older gateway without /netem → section absent
      return null;
    }
  }
  useEffect(() => {
    if (open) fetchNetem();
  }, [open, httpBase]);

  async function applyNetem(id: string) {
    setNetBusy(true);
    setNetMsg(undefined);
    try {
      const res = await fetch(`${httpBase}/netem`, {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify({ profile: id }),
      });
      const body = await res.json();
      if (!res.ok) setNetMsg(`✗ ${body.error ?? res.status}`);
      else setNetem(body);
    } catch (e) {
      setNetMsg(`✗ ${(e as Error).message}`);
    } finally {
      setNetBusy(false);
    }
  }

  // Workload + sweep knobs. maxHz is the one client-side QoS applied to the sweep.
  const [picked, setPicked] = useState<string[]>(["pose"]);
  const [maxHz, setMaxHz] = useState(0);
  const [dur, setDur] = useState(4000);
  const qos: Qos = { maxHz, rateLimit: "server", conflation: "latest" };
  const qosLabel = `maxHz=${maxHz ? `${maxHz}Hz` : "∞"}`;

  const [running, setRunning] = useState(false);
  const [progress, setProgress] = useState("idle");
  const [rows, setRows] = useState<{ label: string; row: BenchRow }[]>([]);
  const [clock, setClock] = useState<{ offsetMs: number; rttMs: number }>();
  const aborted = useRef(false);
  useEffect(() => () => {
    aborted.current = true;
  }, []);

  const profiles = STREAM_PROFILES.filter((p) => picked.includes(p.id));
  const runScenarios = [
    ...profiles.map((p) => ({ name: p.id, topics: p.topics })),
    ...(heavyOn ? [{ name: `heavy:${heavyKind} (≈${offeredMbps.toFixed(0)}MB/s offered)`, topics: [heavyTopic] }] : []),
  ];
  const benchFlowing = discovered.some((t) => t.topic.startsWith("/load/"));

  async function run() {
    if (!client) return;
    setRunning(true);
    aborted.current = false;
    setRows([]);
    // Clock-sync probe first: over WAN the raw recvTs−srcTs is dominated by machine skew; the
    // ping offset (gateway ≈ source clock) makes end-to-end latency meaningful cross-machine.
    setProgress("clock sync…");
    const off = await client.estimateClockOffset().catch(() => undefined);
    setClock(off);
    // Stamp the network condition too — refresh at sweep start (a profile may have self-healed).
    const net = netem?.enabled ? await fetchNetem() : null;
    const netTag = net?.enabled ? ` · net:${net.active}` : "";
    const out: { label: string; row: BenchRow }[] = [];
    for (const scenario of runScenarios) {
      if (aborted.current) break;
      setProgress(`${activeLabel} · ${scenario.name} · ${qosLabel}…`);
      try {
        const r = await measureScenario(client, scenario, dur, true, qos, {
          offsetMs: off?.offsetMs,
        });
        // Stamp the wire that actually carried the run — Auto silently falls back WT→WS, and a
        // result table that doesn't say which wire won isn't reproducible.
        const wire = client.gatewayLabel;
        const wireTag = wire && wire !== activeLabel ? ` (wire: ${wire})` : "";
        out.push({
          label: `${activeLabel}${wireTag}${netTag} · ${qosLabel}`,
          row: { ...r, scenario: scenario.name },
        });
        setRows([...out]);
      } catch (e) {
        setProgress(`${scenario.name} failed: ${String(e).slice(0, 80)}`);
      }
    }
    setProgress(aborted.current ? "stopped" : "done ✓");
    setRunning(false);
  }

  // Markdown export — group rows by label and reuse the SDK's formatMarkdown.
  const stamp = new Date().toISOString().slice(0, 16).replace("T", " ");
  const byLabel = new Map<string, BenchRow[]>();
  for (const { label, row } of rows) (byLabel.get(label) ?? byLabel.set(label, []).get(label)!).push(row);
  const offeredLabel = heavyOn ? `${offeredMbps.toFixed(1)} MB/s offered (${heavyTopic})` : "no generator";
  const clockLabel = clock
    ? `clock offset ${clock.offsetMs.toFixed(1)}ms (rtt ${clock.rttMs.toFixed(1)}ms) corrected`
    : "no clock sync (latency raw)";
  const md = `# dimoscope in-app bench — ${activeLabel}\n\n` +
    `_${stamp} · ${dur}ms/scenario · end-to-end latency · ${clockLabel} · QoS: ${qosLabel} · ${offeredLabel}_\n` +
    `_${globalThis.location?.origin ?? "?"} · ${globalThis.navigator?.userAgent ?? "?"}_\n\n` +
    [...byLabel].map(([label, rs]) => formatMarkdown(label, activeUrl ?? activeLabel, dur, stamp, rs)).join("\n---\n\n");

  return (
    <div className="panel bench-drawer">
      <button type="button" className="bench-drawer-head" onClick={() => setOpen((o) => !o)}>
        <span className="bench-drawer-caret">{open ? "▾" : "▸"}</span>
        Benchmark
        <span className="muted small">— measure the live transport ({activeLabel}) · workload sweep → copy Markdown</span>
      </button>

      {open && (
        <div className="bench-drawer-body">
          <div className="bench-section">
            <div className="bench-label">Workload (STREAM profile)</div>
            <div className="bench-chips">
              {STREAM_PROFILES.map((p) => (
                <button
                  key={p.id}
                  className={`tab ${picked.includes(p.id) ? "tab-active" : ""}`}
                  title={p.hint}
                  onClick={() => setPicked((ps) => ps.includes(p.id) ? ps.filter((x) => x !== p.id) : [...ps, p.id])}
                >
                  {p.id}
                </button>
              ))}
            </div>
            <div className="muted small" style={{ marginTop: 4 }}>
              {profiles.map((p) => `${p.id}: ${p.hint}`).join("  ·  ") || "pick ≥1 workload"}
            </div>
          </div>

          <div className="bench-section">
            <div className="bench-label">Load generator · large stream (GO2Load RPC)</div>
            {!hasRpc
              ? (
                <div className="muted small">
                  No <span className="mono">GO2Load</span> RPC advertised — run{" "}
                  <span className="mono">deno task dog</span> (or <span className="mono">load</span>), to make{" "}
                  <span className="mono">/load/*</span> flow.
                </div>
              )
              : (
                <>
                  <div className="bench-row">
                    <span className="muted small">kind</span>
                    {(["image", "cloud"] as const).map((k) => (
                      <button
                        key={k}
                        className={`tab ${heavyKind === k ? "tab-active" : ""}`}
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
                    <span className="muted small">≈{offeredMbps.toFixed(1)} MB/s offered</span>
                    <button className="tab" disabled={genBusy} onClick={() => applyGen(true)}>
                      {heavyOn ? "Re-apply" : "▶ Start load"}
                    </button>
                    <button className="tab" disabled={genBusy || !heavyOn} onClick={() => applyGen(false)}>
                      Stop load
                    </button>
                    {heavyOn && <span className="badge">{heavyTopic} live</span>}
                    {genMsg && <span className="muted small" title={genMsg}>{genMsg.slice(0, 60)}</span>}
                  </div>
                </>
              )}
          </div>

          {netem?.enabled && (
            <div className="bench-section">
              <div className="bench-label">
                Network · server-side netem ({gateway})
              </div>
              <div className="bench-chips">
                {netem.profiles.map((p) => (
                  <button
                    key={p.id}
                    className={`tab ${netem.active === p.id ? "tab-active" : ""}`}
                    title={p.desc}
                    disabled={netBusy}
                    onClick={() => applyNetem(p.id)}
                  >
                    {p.label}
                  </button>
                ))}
              </div>
              <div className="bench-row">
                <span className="muted small">outage</span>
                {netem.outages.map((o) => (
                  <button
                    key={o.id}
                    className="tab"
                    title={o.desc}
                    disabled={netBusy}
                    onClick={() => applyNetem(o.id)}
                  >
                    {o.label}
                  </button>
                ))}
                {netMsg && <span className="muted small" title={netMsg}>{netMsg.slice(0, 60)}</span>}
              </div>
              <div className="muted small">
                tc on the gateway egress (ports 8080/8443 only) · self-heals in{" "}
                {Math.round(netem.healsInS / 60)} min · apply loss <em>after</em>{" "}
                the transport is connected — QUIC handshakes fail under loss
              </div>
            </div>
          )}

          <div className="bench-section">
            <div className="bench-label">QoS · client-side (applied to the sweep)</div>
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
              <button
                className="tab"
                onClick={run}
                disabled={running || !client || !runScenarios.length}
              >
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
            {!benchFlowing && !heavyOn && (
              <div className="muted small">
                No <span className="mono">/load/*</span> flowing — Start load above, or run{" "}
                <span className="mono">deno task dog</span>, so the sweep has data to measure.
              </div>
            )}
          </div>

          {rows.length > 0 && (
            <div className="bench-section">
              <div className="bench-label">Results · end-to-end (publish→browser)</div>
              <table className="stats" style={{ width: "100%" }}>
                <thead>
                  <tr>
                    <th>config</th>
                    <th>scenario</th>
                    <th>hz</th>
                    <th>kB/s</th>
                    <th>p50</th>
                    <th>p95</th>
                    <th>p99</th>
                    <th>loss%</th>
                    <th>late%</th>
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
                      <td>{row.latePct}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
        </div>
      )}
    </div>
  );
}
