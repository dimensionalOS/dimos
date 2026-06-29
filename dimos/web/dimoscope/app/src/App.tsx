// dimoscope — the example app for @dimos/topics + @dimos/react.
// Shows: live topic discovery, a fused 2D WorldView, a Pose readout, safe
// teleop, and a live StatsBar (hz / bandwidth / latency per topic).
import { useState } from "react";
import { useStatus, useTopics, useTopicLatest } from "@dimos/react";
import { WorldView } from "./panels/WorldView";
import { PoseReadout } from "./panels/PoseReadout";
import { TeleopPad } from "./panels/TeleopPad";
import { StatsBar } from "./panels/StatsBar";
import { RerunPanel } from "./panels/RerunPanel";

function Inspector({ topic }: { topic: string }) {
  const { data, meta } = useTopicLatest<any>(topic, { maxHz: 4 });
  const pretty = JSON.stringify(
    data,
    (_k, v) => (typeof v === "bigint" ? v.toString() : v instanceof Uint8Array ? `<${v.length} bytes>` : v),
    2,
  );
  return (
    <div className="panel">
      <div className="panel-title">
        Inspector · {topic} {meta ? `· ${meta.sizeBytes}B` : ""}
      </div>
      <pre className="json">{pretty?.slice(0, 4000) ?? "waiting…"}</pre>
    </div>
  );
}

export function App() {
  const topics = useTopics();
  const status = useStatus();
  const [selected, setSelected] = useState<string | null>(null);

  return (
    <div className="layout">
      <header className="topbar">
        <b>dimoscope</b>
        <span className={`status status-${status}`}>● {status}</span>
        <span className="badge">gateway · Bun↔LCM</span>
        <span className="muted">DimOS topics in the browser — subscribe · visualize · teleop</span>
      </header>

      <aside className="sidebar">
        <div className="sidebar-title">Topics ({topics.length})</div>
        {topics.length === 0 && (
          <div className="muted small">discovering… is the gateway + a source running?</div>
        )}
        {topics.map((t) => (
          <button
            key={t.topic}
            className={`topic ${selected === t.topic ? "topic-sel" : ""}`}
            onClick={() => setSelected(selected === t.topic ? null : t.topic)}
          >
            <div className="topic-name">{t.topic}</div>
            <div className="topic-type">{t.type}</div>
          </button>
        ))}
      </aside>

      <main className="main">
        <WorldView />
        <div className="side-col">
          <PoseReadout />
          <TeleopPad />
          {selected ? <Inspector topic={selected} /> : <StatsBar />}
        </div>
        <RerunPanel />
      </main>
    </div>
  );
}
