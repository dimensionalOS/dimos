// dimoscope — the example app for @dimos/topics + @dimos/react.
// Shows: live topic discovery, a fused 2D WorldView, a Pose readout, safe
// teleop, and a live StatsBar (hz / bandwidth / latency per topic).
import { useState } from "react";
import {
  type MediaMode,
  SubscribeBar,
  useDimosClient,
  useServers,
  useStatus,
  useTopicLatest,
  useTopics,
} from "@dimos/react";
import { WorldView } from "./panels/WorldView";
import { CameraView } from "./panels/CameraView";
import { PoseReadout } from "./panels/PoseReadout";
import { TeleopPad } from "./panels/TeleopPad";
import { StatsBar } from "./panels/StatsBar";
import { RerunPanel } from "./panels/RerunPanel";
import { CommandsPanel } from "./panels/CommandsPanel";
import { BenchPanel } from "./panels/BenchPanel";
import { BenchTab } from "./panels/BenchTab";

function Inspector({ topic }: { topic: string }) {
  const { data, meta } = useTopicLatest<any>(topic, { maxHz: 4 });
  const pretty = JSON.stringify(
    data,
    (
      _k,
      v,
    ) => (typeof v === "bigint"
      ? v.toString()
      : v instanceof Uint8Array
      ? `<${v.length} bytes>`
      : v),
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
  const { servers, activeId, setActiveId } = useServers();
  const [selected, setSelected] = useState<string | null>(null);
  const [tab, setTab] = useState<"2d" | "3d" | "bench">("2d");
  const [mediaMode, setMediaMode] = useState<MediaMode>("auto");

  return (
    <div className="layout">
      <header className="topbar">
        <span
          className="wordmark"
          title="DimOS topics in the browser — subscribe · visualize · teleop"
        >
          dimo<span>scope</span>
        </span>
        <span className={`status status-${status}`}>{status}</span>
        {/* mode switch — console (WorldView) ⇄ full-page 3D (Rerun) */}
        <div className="tabs" style={{ marginLeft: 6 }}>
          <button
            className={`tab ${tab === "2d" ? "tab-active" : ""}`}
            onClick={() => setTab("2d")}
          >
            WorldView
          </button>
          <button
            className={`tab ${tab === "3d" ? "tab-active" : ""}`}
            onClick={() => setTab("3d")}
          >
            Rerun · 3D
          </button>
          <button
            className={`tab ${tab === "bench" ? "tab-active" : ""}`}
            onClick={() => setTab("bench")}
          >
            Bench
          </button>
        </div>
        <div className="topbar-right">
          {servers.length > 1 && (
            <select
              className="server-select"
              value={activeId ?? ""}
              onChange={(e) => setActiveId(e.target.value)}
              title="active transport / server"
            >
              {servers.map((s) => (
                <option key={s.id} value={s.id}>
                  {s.label}
                </option>
              ))}
            </select>
          )}
          <select
            className="server-select"
            value={mediaMode}
            onChange={(e) => setMediaMode(e.target.value as MediaMode)}
            title="camera media mode (A/B the bandwidth win)"
          >
            <option value="auto">cam: auto</option>
            <option value="webrtc">cam: webrtc</option>
            <option value="webcodecs">cam: webcodecs</option>
            <option value="jpeg">cam: jpeg</option>
          </select>
          <span className="badge">gateway · {label ?? "connecting…"}</span>
        </div>
      </header>

      {
        /* ── WorldView mode = the operator console (camera center + side panels). It UNMOUNTS in Rerun
          mode → frees its camera + lidar/odom/map subscriptions (Rerun has its own gRPC feed). ── */
      }
      {tab === "2d" && (
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
      )}
      {tab === "2d" && (
        <main className="main">
          <div className="center-col">
            {/* Camera is the PRIMARY view — you drive looking at it (⛶ fullscreen). */}
            <CameraView mode={mediaMode} primary />
          </div>
          <div className="side-col">
            {
              /* Spatial 2D (lidar/map): per-layer on/off + live bandwidth (toggle lidar off → drop
                ~2 MB/s). The WorldView/Rerun mode switch lives in the topbar now. */
            }
            <div className="spatial-viz">
              <WorldView />
            </div>
            <PoseReadout />
            <TeleopPad />
            <CommandsPanel />
            <BenchPanel />
            <SubscribeBar />
            {selected ? <Inspector topic={selected} /> : <StatsBar />}
          </div>
        </main>
      )}

      {
        /* ── Rerun = full-page 3D. Always mounted (warm) so switching is instant + avoids a cold gRPC
          re-stream; shown only when the topbar mode = Rerun. ── */
      }
      <div className="rerun-full" style={tab === "3d" ? undefined : { display: "none" }}>
        <RerunPanel active={tab === "3d"} />
      </div>

      {
        /* ── Bench = full-page interactive benchmark. MOUNT/UNMOUNT (unlike Rerun): leaving the tab
          drops its monitor subscriptions + aborts an in-flight sweep, so it never floods in the
          background. ── */
      }
      {tab === "bench" && (
        <div className="bench-full">
          <BenchTab />
        </div>
      )}
    </div>
  );
}
