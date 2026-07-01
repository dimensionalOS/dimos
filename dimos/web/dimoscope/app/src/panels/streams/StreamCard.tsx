// StreamCard — one subscribed topic, live. Shows its QoS lane (from the SDK's own qos.ts), live
// hz/kB·s/latency/gap + a rate sparkline, a rolling display-throttled feed of the actual messages
// (expandable to full JSON), AND per-topic QoS controls wired the idiomatic way (resolveQos → setQos).
import { useEffect, useState } from "react";
import { jsonPretty, useCaps, useDimosClient, useTopicFeed, useTopicStats } from "@dimos/react";
import { defaultLane, type Lane, LANES, resolveQos } from "@dimos/web";
import { Sparkline } from "../../widgets/Sparkline";

const LANE_OPTS: Lane[] = ["command", "sensor", "default", "bulk"];
const HZ_OPTS = [0, 5, 20, 60, 120];

const pad = (n: number, w = 2) => String(n).padStart(w, "0");
const fmtTime = (ms: number) => {
  const d = new Date(ms);
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}.${pad(d.getMilliseconds(), 3)}`;
};
const fmtSize = (b: number) =>
  b >= 1e6 ? `${(b / 1e6).toFixed(1)}MB` : b >= 1000 ? `${(b / 1000).toFixed(b >= 1e5 ? 0 : 1)}kB` : `${b}B`;

export function StreamCard({ topic, type, onRemove, boost = false }: {
  topic: string;
  type: string;
  onRemove: () => void;
  /** Contention mode: bump non-bulk lanes to `critical` so they win the scheduler under load. */
  boost?: boolean;
}) {
  const client = useDimosClient();
  const caps = useCaps();
  const feed = useTopicFeed(topic, { maxRows: 40 });
  const stats = useTopicStats(topic, 250);
  const [hist, setHist] = useState<number[]>([]);
  const [expanded, setExpanded] = useState(false);
  const [laneOverride, setLaneOverride] = useState<Lane | "">("");
  const [maxHz, setMaxHz] = useState(0);

  const autoLane = defaultLane(topic, type);
  const lane = laneOverride || autoLane;
  const preset = LANES[lane];
  // Does the active transport's scheduler honor priority/reliability/depth? (WS/WebTransport yes;
  // sse/poll/zenoh-ts advertise none → only maxHz bites, client-side.)
  const schedHonored = (caps?.qos?.transport?.length ?? 0) > 0;
  const live = !!stats && stats.hz > 0;

  // Contention mode boosts non-bulk lanes to `critical` (bulk stays low → it sheds first under load).
  const boosted = boost && lane !== "bulk";
  const effPrio = boosted ? "critical" : preset.priority;

  // Declare this topic's QoS the way the framework intends: resolveQos(lane preset + overrides) → setQos.
  // Runs on mount and whenever a knob changes; the shared Topic handle re-issues the wire subscribe.
  useEffect(() => {
    if (!client) return;
    client.topic(topic).setQos(resolveQos(topic, type, {
      lane,
      ...(maxHz > 0 ? { maxHz } : {}),
      ...(boosted ? { priority: "critical" as const } : {}),
    }));
  }, [client, topic, type, lane, maxHz, boosted]);

  useEffect(() => {
    if (!stats) return;
    setHist((h) => [...h.slice(-59), stats.bytesPerSec / 1000]);
  }, [stats]);

  const kb = stats ? stats.bytesPerSec / 1000 : 0;
  return (
    <div className={`panel stream-card ${boosted ? "stream-boosted" : ""}`}>
      <div className="stream-head">
        <span
          className="stream-dot"
          style={{ color: live ? "var(--ok)" : "var(--muted)" }}
          title={live ? "receiving messages" : "subscribed — waiting for the first message"}
        >
          ●
        </span>
        <span className="stream-topic" title={topic}>{topic}</span>
        <span
          className={`lane lane-${lane}`}
          title={`QoS lane ${laneOverride ? "(override)" : "(qos.ts defaultLane)"}: ${lane} — priority ${preset.priority}, ${preset.conflation}`}
        >
          {lane}
        </span>
        <button type="button" className="tab stream-x" onClick={onRemove} title="unsubscribe">×</button>
      </div>
      <div className="stream-type muted small">{type || "—"}</div>

      <div className="stream-metrics">
        <div className="metric"><b>{stats?.hz ?? 0}</b><span>hz</span></div>
        <div className="metric"><b>{kb.toFixed(1)}</b><span>kB/s</span></div>
        <div className="metric">
          <b>{stats?.lastLatencyMs != null ? stats.lastLatencyMs.toFixed(1) : "–"}</b><span>ms</span>
        </div>
        <div
          className="metric"
          title="source-sequence gap: share of published messages not delivered here — rises with latest-wins conflation (sensor/bulk lanes) or link shedding under load"
        >
          <b>{feed.lossPct != null ? feed.lossPct.toFixed(feed.lossPct >= 10 ? 0 : 1) : "–"}</b>
          <span>gap%</span>
        </div>
        <Sparkline data={hist} color="var(--signal)" width={104} height={26} />
      </div>

      <div className="stream-qos">
        <label className="qos-ctl">
          <span className="qos-lbl">lane</span>
          <select
            className="qos-select"
            value={laneOverride}
            onChange={(e) => setLaneOverride(e.target.value as Lane | "")}
            title="QoS lane — the ROS 2-style preset (priority + conflation) the gateway scheduler honors"
          >
            <option value="">auto · {autoLane}</option>
            {LANE_OPTS.map((l) => <option key={l} value={l}>{l}</option>)}
          </select>
        </label>
        <div className="qos-ctl">
          <span className="qos-lbl">maxHz</span>
          <div className="qos-hz">
            {HZ_OPTS.map((h) => (
              <button
                key={h}
                type="button"
                className={`tab ${maxHz === h ? "tab-active" : ""}`}
                onClick={() => setMaxHz(h)}
                title={h === 0 ? "unlimited" : `cap delivery to ${h} Hz`}
              >
                {h === 0 ? "∞" : h}
              </button>
            ))}
          </div>
        </div>
        <div className="qos-eff muted small">
          {schedHonored ? `prio ${effPrio}${boosted ? " ⚡" : ""} · ${preset.conflation}` : `client-only · maxHz`}
          {stats && stats.dropped > 0 ? ` · ${stats.dropped} dropped` : ""}
        </div>
      </div>

      <div className="stream-feed">
        {feed.rows.length === 0 && <div className="muted small">waiting for messages…</div>}
        {feed.rows.slice().reverse().map((r) => (
          <div className="feed-row" key={r.id} title={r.preview}>
            <span className="feed-t">{fmtTime(r.recvTs)}</span>
            <span className="feed-sz">{fmtSize(r.sizeBytes)}</span>
            {r.seq != null && <span className="feed-seq">#{r.seq}</span>}
            <span className="feed-body">{r.preview}</span>
          </div>
        ))}
      </div>

      <button type="button" className="tab stream-json-btn" onClick={() => setExpanded((e) => !e)}>
        {expanded ? "hide JSON" : "latest JSON"}
      </button>
      {expanded && <pre className="json">{feed.latest ? jsonPretty(feed.latest.data) : "waiting…"}</pre>}
    </div>
  );
}
