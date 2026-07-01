// dimoscope — the example app for @dimos/web + @dimos/react.
import { useState } from "react";
import { type MediaMode, SubscribeBar } from "@dimos/react";
import {
  useDimosClient,
  useServers,
  useStatus,
  useTopicLatest,
  useTopics,
} from "./dimos";
import { WorldView } from "./panels/WorldView";
import { CameraView } from "./panels/CameraView";
import { PoseReadout } from "./panels/PoseReadout";
import { TeleopPad } from "./panels/TeleopPad";
import { StatsBar } from "./panels/StatsBar";
import { CommandsPanel } from "./panels/CommandsPanel";
import { StreamsTab } from "./panels/streams/StreamsTab";
import { BenchDrawer } from "./panels/BenchDrawer";
import { useGateway } from "./gateway";

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
  const [tab, setTab] = useState<"2d" | "streams">("2d");
  const [mediaMode, setMediaMode] = useState<MediaMode>("auto");
  const { gateway, setGateway } = useGateway();
  const [gwText, setGwText] = useState(gateway);

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
        <div className="tabs" style={{ marginLeft: 6 }}>
          <button
            className={`tab ${tab === "2d" ? "tab-active" : ""}`}
            onClick={() => setTab("2d")}
          >
            WorldView
          </button>
          <button
            className={`tab ${tab === "streams" ? "tab-active" : ""}`}
            onClick={() => setTab("streams")}
          >
            Topics
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
          <input
            className="gw-input"
            value={gwText}
            onChange={(e) => setGwText(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === "Enter") {
                setGateway(gwText);
                (e.target as HTMLInputElement).blur();
              }
            }}
            onBlur={() => setGateway(gwText)}
            placeholder="host:port"
            title="gateway server (host:port) — reconnects on change; saved to this browser"
            spellCheck={false}
          />
          <span className="badge">gateway · {label ?? "connecting…"}</span>
        </div>
      </header>

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
            <div className="spatial-viz">
              <WorldView />
            </div>
            <PoseReadout />
            <TeleopPad />
            <CommandsPanel />
            <SubscribeBar />
            {selected ? <Inspector topic={selected} /> : <StatsBar />}
          </div>
        </main>
      )}

      {tab === "streams" && (
        <div className="streams-full">
          <StreamsTab />
          <BenchDrawer />
        </div>
      )}
    </div>
  );
}
