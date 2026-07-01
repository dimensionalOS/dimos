// StreamsTab — subscribe to any topic (by name or a scenario preset) and watch its messages stream in
// with per-topic QoS controls. The live companion to the Bench tab's quantitative sweep.
import { useState } from "react";
import { useTopics } from "../../dimos";
import { StreamCard } from "./StreamCard";

// The scenario namespaces (scenarios/{nav,arm,cam}.py) + the go2-load blueprint — subscribe a whole
// profile in one click. Listed by name so a preset works even before the publisher is discovered.
const PRESETS: Record<string, string[]> = {
  nav: ["/nav/pose", "/nav/path", "/nav/cloud", "/nav/map"],
  arm: ["/arm/joint_states", "/arm/ee_pose", "/arm/imu", "/arm/trajectory"],
  cam: ["/cam/rgb", "/cam/depth", "/cam/points", "/cam/detections"],
  // go2-load: multi-rate streams alongside the teleoperable go2 dimsim (distinct Hz per lane).
  load: ["/load/fast", "/load/mid", "/load/slow", "/load/grid", "/load/cloud"],
};

export function StreamsTab() {
  const topics = useTopics();
  const [subs, setSubs] = useState<string[]>([]);
  const [text, setText] = useState("");
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
          <div className="stream-grid">
            {subs.map((t) => (
              <StreamCard key={t} topic={t} type={typeOf(t)} onRemove={() => remove(t)} />
            ))}
          </div>
        )}
    </div>
  );
}
