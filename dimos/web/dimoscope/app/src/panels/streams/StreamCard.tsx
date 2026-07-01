// StreamCard — one subscribed topic, live. Shows its QoS lane, live hz/kB·s/latency/gap + a rate
// sparkline, a rolling display-throttled feed of the actual messages (expandable to full JSON), AND
// the FULL per-topic QoS control surface the SDK exposes, wired the idiomatic way (resolveQos → setQos).
import { useEffect, useState } from "react";
import { jsonPretty, useTopicFeed } from "@dimos/react";
import { useCaps, useDimosClient, useTopicStats } from "../../dimos";
import { defaultLane, type Lane, LANES, type Qos, resolveQos } from "@dimos/web";
import { Sparkline } from "../../widgets/Sparkline";

// The knobs the SDK's `Qos` exposes (packages/web/src/types.ts) — every one is controllable here.
const LANE_OPTS: (Lane | "")[] = ["", "command", "sensor", "default", "bulk"];
const HZ_OPTS = [0, 5, 20, 60, 120];
const RATELIMIT_OPTS = ["server", "client"] as const;
const CONFLATION_OPTS = ["latest", "all"] as const;
const RELIABILITY_OPTS = ["reliable", "best-effort"] as const;
const DEPTH_OPTS = [1, 5, 10, 16];
const PRIORITY_OPTS = ["low", "normal", "high", "critical"] as const;

const pad = (n: number, w = 2) => String(n).padStart(w, "0");
const fmtTime = (ms: number) => {
  const d = new Date(ms);
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}.${pad(d.getMilliseconds(), 3)}`;
};
const fmtSize = (b: number) =>
  b >= 1e6 ? `${(b / 1e6).toFixed(1)}MB` : b >= 1000 ? `${(b / 1000).toFixed(b >= 1e5 ? 0 : 1)}kB` : `${b}B`;

// A compact segmented button group — one row of a QoS knob's options; highlights the active value.
function Seg<T extends string | number>({ options, value, onChange, disabled, render }: {
  options: readonly T[];
  value: T | undefined;
  onChange: (v: T) => void;
  disabled?: boolean;
  render?: (v: T) => string;
}) {
  return (
    <div className="qos-seg">
      {options.map((o) => (
        <button
          key={String(o)}
          type="button"
          disabled={disabled}
          className={`tab ${value === o ? "tab-active" : ""}`}
          onClick={() => onChange(o)}
        >
          {render ? render(o) : String(o)}
        </button>
      ))}
    </div>
  );
}

export function StreamCard({ topic, type, onRemove }: {
  topic: string;
  type: string;
  onRemove: () => void;
}) {
  const client = useDimosClient();
  const caps = useCaps();
  const feed = useTopicFeed(topic, { maxRows: 40 });
  const stats = useTopicStats(topic, 250);
  const [hist, setHist] = useState<number[]>([]);
  const [peakHz, setPeakHz] = useState(0);
  const [expanded, setExpanded] = useState(false);

  // Control state: a base `lane` preset + explicit per-knob overrides on top of it (`over`), plus the
  // rate cap which is independent of the lane. eff = resolveQos merges LANES[lane] with the overrides.
  const [laneOverride, setLaneOverride] = useState<Lane | "">("");
  const [maxHz, setMaxHz] = useState(0);
  const [rateLimit, setRateLimit] = useState<NonNullable<Qos["rateLimit"]>>("server");
  const [over, setOver] = useState<Partial<Pick<Qos, "conflation" | "reliability" | "depth" | "priority">>>({});

  const autoLane = defaultLane(topic, type);
  const activeLane = laneOverride || autoLane;
  const eff = resolveQos(topic, type, {
    lane: activeLane,
    ...over,
    ...(maxHz > 0 ? { maxHz } : {}),
    rateLimit,
  });

  // Which fields the active transport actually honors (WS/WebTransport = scheduler + server downsample;
  // sse/poll/zenoh-ts advertise none → priority/reliability/depth/server-rateLimit are client-emulated).
  const serverRate = caps?.qos?.maxHz === "server";
  const schedHonored = (caps?.qos?.transport?.length ?? 0) > 0;
  const live = !!stats && stats.hz > 0;

  // Declare QoS the way the framework intends: resolveQos(lane preset + overrides) → setQos. Re-applied
  // whenever any knob changes; the shared Topic handle re-issues the wire subscribe with the new QoS.
  useEffect(() => {
    if (!client) return;
    client.topic(topic).setQos(resolveQos(topic, type, {
      lane: activeLane,
      ...over,
      ...(maxHz > 0 ? { maxHz } : {}),
      rateLimit,
    }));
  }, [client, topic, type, activeLane, maxHz, rateLimit, over]);

  useEffect(() => {
    if (!stats) return;
    setHist((h) => [...h.slice(-59), stats.bytesPerSec / 1000]);
    if (stats.hz > peakHz) setPeakHz(stats.hz);
  }, [stats, peakHz]);

  const pickLane = (l: Lane | "") => {
    setLaneOverride(l);
    setOver({}); // a fresh preset — clear per-knob overrides so the controls reflect the lane
  };
  const setKnob = <K extends keyof typeof over>(k: K, v: (typeof over)[K]) =>
    setOver((o) => ({ ...o, [k]: v }));

  const kb = stats ? stats.bytesPerSec / 1000 : 0;
  const summary = `${eff.reliability}, ${eff.conflation}-wins, ${eff.priority} priority, keep-last ${eff.depth}` +
    (maxHz > 0 ? `, ≤${maxHz} Hz (${rateLimit})` : "");

  return (
    <div className="panel stream-card">
      <div className="stream-head">
        <span
          className="stream-dot"
          style={{ color: live ? "var(--ok)" : "var(--muted)" }}
          title={live ? "receiving messages" : "subscribed — waiting for the first message"}
        >
          ●
        </span>
        <span className="stream-topic" title={topic}>{topic}</span>
        <span className={`lane lane-${activeLane}`} title={`QoS lane: ${activeLane}`}>{activeLane}</span>
        <button type="button" className="tab stream-x" onClick={onRemove} title="unsubscribe">×</button>
      </div>
      <div className="stream-type muted small">{type || "—"}</div>

      <div className="stream-metrics">
        <div className="metric" title={peakHz > (stats?.hz ?? 0) + 1 ? `peak seen ~${peakHz} Hz` : ""}>
          <b>{stats?.hz ?? 0}</b><span>hz{peakHz > (stats?.hz ?? 0) + 1 ? ` /${peakHz}` : ""}</span>
        </div>
        <div className="metric"><b>{kb.toFixed(1)}</b><span>kB/s</span></div>
        <div className="metric">
          <b>{stats?.lastLatencyMs != null ? stats.lastLatencyMs.toFixed(1) : "–"}</b><span>ms</span>
        </div>
        <div
          className="metric"
          title="source-sequence gap: share of published messages not delivered here — rises with latest-wins conflation or link shedding under load"
        >
          <b>{feed.lossPct != null ? feed.lossPct.toFixed(feed.lossPct >= 10 ? 0 : 1) : "–"}</b>
          <span>gap%</span>
        </div>
        <div className="metric" title="messages the client-side rate-limit dropped locally (rateLimit=client)">
          <b>{stats?.dropped ?? 0}</b><span>drop</span>
        </div>
        <Sparkline data={hist} color="var(--signal)" width={96} height={24} />
      </div>

      {/* full QoS surface — grouped rate / delivery / scheduler; each highlights the effective value */}
      <div className="stream-qos">
        <div className="qos-row">
          <span className="qos-lbl" title="ROS 2-style preset — sets the knobs below in one click">lane</span>
          <Seg
            options={LANE_OPTS}
            value={laneOverride}
            onChange={pickLane}
            render={(l) => (l === "" ? `auto·${autoLane}` : l)}
          />
        </div>

        <div className="qos-grp-lbl">rate — visible now</div>
        <div className="qos-row">
          <span className="qos-lbl" title="cap delivery to N Hz (∞ = unlimited)">maxHz</span>
          <Seg options={HZ_OPTS} value={maxHz} onChange={setMaxHz} render={(h) => (h === 0 ? "∞" : String(h))} />
        </div>
        <div className="qos-row">
          <span className="qos-lbl" title="server = gateway downsamples (fewer bytes on the wire); client = keep bytes, drop locally">
            limit
          </span>
          <Seg options={RATELIMIT_OPTS} value={rateLimit} onChange={(v) => setRateLimit(v)} disabled={!serverRate} />
          {!serverRate && <span className="muted small qos-note">client-only transport</span>}
        </div>

        <div className="qos-grp-lbl">delivery{schedHonored ? "" : " — client-emulated on this transport"}</div>
        <div className="qos-row">
          <span className="qos-lbl" title="latest = drop stale under a cap; all = deliver every message">conflate</span>
          <Seg options={CONFLATION_OPTS} value={eff.conflation} onChange={(v) => setKnob("conflation", v)} />
        </div>
        <div className="qos-row">
          <span className="qos-lbl" title="reliable = keep-last buffer; best-effort = conflate, shed first under load">reliab.</span>
          <Seg
            options={RELIABILITY_OPTS}
            value={eff.reliability}
            onChange={(v) => setKnob("reliability", v)}
            disabled={!schedHonored}
          />
        </div>
        <div className="qos-row">
          <span className="qos-lbl" title="keep-last deque size the gateway buffers per topic">depth</span>
          <Seg
            options={DEPTH_OPTS}
            value={eff.depth}
            onChange={(v) => setKnob("depth", v)}
            disabled={!schedHonored}
          />
        </div>

        <div className="qos-grp-lbl">scheduler{schedHonored ? " — matters under load" : " — not honored on this transport"}</div>
        <div className="qos-row">
          <span className="qos-lbl" title="the gateway drains higher priority first and sheds lower first under contention">priority</span>
          <Seg
            options={PRIORITY_OPTS}
            value={eff.priority}
            onChange={(v) => setKnob("priority", v)}
            disabled={!schedHonored}
          />
        </div>

        <div className="qos-summary muted small" title="the effective QoS this topic is subscribed with">
          → {summary}
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
