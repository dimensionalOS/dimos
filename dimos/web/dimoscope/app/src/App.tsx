// dimoscope — the example app for @dimos/topics + @dimos/react.
// Shows: live topic discovery, a fused 2D WorldView, a Pose readout, safe
// teleop, and a live StatsBar (hz / bandwidth / latency per topic).
import { useState } from "react";
import { useStatus, useTopics, useTopicLatest, useDimosClient } from "@dimos/react";
import { WorldView } from "./panels/WorldView";
import { CameraView } from "./panels/CameraView";
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
  const label = useDimosClient()?.gatewayLabel;
  const [selected, setSelected] = useState<string | null>(null);
  const [tab, setTab] = useState<"2d" | "3d">("2d");

  return (
    <div className="layout">
      <header className="topbar">
        <b>dimoscope</b>
        <span className={`status status-${status}`}>● {status}</span>
        <span className="badge">gateway · {label ?? "connecting…"}</span>
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
        <div className="center-col">
          <div className="tabs">
            <button
              className={`tab ${tab === "2d" ? "tab-active" : ""}`}
              onClick={() => setTab("2d")}
            >
              WorldView · 2D
            </button>
            <button
              className={`tab ${tab === "3d" ? "tab-active" : ""}`}
              onClick={() => setTab("3d")}
            >
              Rerun · 3D
            </button>
          </div>
          {/* WorldView is cheap to unmount (its effects just unsubscribe), so we
              UNMOUNT it on 3D → frees lidar/odom/map there. Rerun is NOT cheap to
              remount (a cold grpc re-stream of the recording freezes the renderer),
              so keep it WARM-mounted and merely hide it on 2D; its render throttles
              while display:none and it switches in instantly. */}
          <div className="center-viz" style={tab === "2d" ? undefined : { display: "none" }}>
            {tab === "2d" && <WorldView />}
          </div>
          <div className="center-viz" style={tab === "3d" ? undefined : { display: "none" }}>
            <RerunPanel active={tab === "3d"} />
          </div>
        </div>
        <div className="side-col">
          {/* On 3D, Rerun shows the camera in its own viewport — skip the ~11 MB/s side feed. */}
          {tab === "2d" && <CameraView />}
          <PoseReadout />
          <TeleopPad />
          {selected ? <Inspector topic={selected} /> : <StatsBar />}
        </div>
      </main>
    </div>
  );
}
