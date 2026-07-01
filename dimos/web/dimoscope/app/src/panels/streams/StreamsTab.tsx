// StreamsTab — the full-page live-stream + QoS showcase. Subscribe to any topic (by name or a
// scenario preset) and watch its messages stream in, each with its QoS lane and live metrics.
// This is the "watch it live / tune QoS" companion to the Bench tab's quantitative sweep.
import { useState } from "react";
import { useTopics } from "@dimos/react";
import { StreamCard } from "./StreamCard";

// The scenario namespaces (scenarios/{nav,arm,cam}.py) + the go2-scope blueprint — subscribe a whole
// profile in one click. Listed by name so a preset works even before the publisher is discovered.
const PRESETS: Record<string, string[]> = {
  nav: ["/nav/pose", "/nav/path", "/nav/cloud", "/nav/map"],
  arm: ["/arm/joint_states", "/arm/ee_pose", "/arm/imu", "/arm/trajectory"],
  cam: ["/cam/rgb", "/cam/depth", "/cam/points", "/cam/detections"],
  // go2-scope: multi-rate streams alongside the teleoperable go2 dimsim (distinct Hz per lane).
  scope: ["/scope/fast", "/scope/mid", "/scope/slow", "/scope/grid", "/scope/cloud"],
};

export function StreamsTab() {
  const topics = useTopics();
  const [subs, setSubs] = useState<string[]>([]);
  const [text, setText] = useState("");
  const [contention, setContention] = useState(false);
  const typeOf = (t: string) => topics.find((x) => x.topic === t)?.type ?? "";
  const add = (names: string[]) =>
    setSubs((prev) => {
      const next = [...prev];
      for (const n of names.map((x) => x.trim())) if (n && !next.includes(n)) next.push(n);
      return next;
    });
  const remove = (t: string) => setSubs((s) => s.filter((x) => x !== t));

  return (
    <div className="streams-tab">
      <div className="panel">
        <div className="panel-title">Streams · subscribe &amp; watch live · QoS lane per topic</div>
        <form
          className="stream-subscribe"
          onSubmit={(e) => {
            e.preventDefault();
            add([text]);
            setText("");
          }}
        >
          <input
            list="stream-topic-names"
            placeholder="/nav/…  subscribe by name"
            value={text}
            onChange={(e) => setText(e.target.value)}
          />
          <datalist id="stream-topic-names">
            {topics.map((t) => <option key={t.topic} value={t.topic} />)}
          </datalist>
          <button type="submit" className="tab">+ subscribe</button>
        </form>
        <div className="bench-row" style={{ marginTop: 8 }}>
          <span className="bench-label">presets</span>
          {Object.keys(PRESETS).map((k) => (
            <button key={k} type="button" className="tab" onClick={() => add(PRESETS[k])}>{k}</button>
          ))}
          <button type="button" className="tab" onClick={() => add(topics.map((t) => t.topic))}>
            all discovered
          </button>
          {subs.length > 0 && (
            <button type="button" className="tab" onClick={() => setSubs([])}>clear</button>
          )}
          <span className="muted small">{subs.length} subscribed</span>
        </div>
      </div>

      {subs.length === 0
        ? (
          <div className="muted small" style={{ padding: "8px 2px" }}>
            No streams yet — subscribe by name or pick a preset. (Run a scenario first, e.g.{" "}
            <code>deno task scope:nav</code>.)
          </div>
        )
        : (
          <>
            <div className={`contention-bar ${contention ? "contention-on" : ""}`}>
              <div className="contention-head">
                <button
                  type="button"
                  className={`tab ${contention ? "tab-active" : ""}`}
                  onClick={() => setContention((c) => !c)}
                >
                  ⚡ Contention {contention ? "ON" : "OFF"}
                </button>
                <span className="contention-title">
                  priority scheduler — high-priority lanes drain first, bulk sheds under load
                </span>
              </div>
              <div className="muted small contention-hint">
                {contention
                  ? "SENSOR / COMMAND / DEFAULT topics boosted to critical ⚡ (bulk stays low). Now throttle the link to see the scheduler shed: DevTools → Network → add a custom profile (~300 kb/s) → refresh. Watch the BULK cards (cloud/map) collapse toward 0 Hz with rising gap% while the boosted cards stay crisp."
                  : "Loopback is unconstrained, so the scheduler rarely needs to shed. Turn this on to boost non-bulk topics to critical, then throttle the link (DevTools → Network → custom ~300 kb/s → refresh) to watch bulk shed first while sensors survive."}
              </div>
            </div>
            <div className="stream-grid">
              {subs.map((t) => (
                <StreamCard
                  key={t}
                  topic={t}
                  type={typeOf(t)}
                  onRemove={() => remove(t)}
                  boost={contention}
                />
              ))}
            </div>
          </>
        )}
    </div>
  );
}
