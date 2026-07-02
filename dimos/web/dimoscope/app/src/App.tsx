// dimoscope — the example app for @dimos/web + @dimos/react.
import { useEffect, useRef, useState } from "react";
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
import { BenchDrawer, hasBenchParams } from "./panels/bench/BenchDrawer";
import { TopbarNetem } from "./panels/TopbarNetem";
import { normalizeGateway, recentGateways, useGateway } from "./gateway";
import { NetemProvider } from "./netem";
import { getParam, setUrlParam } from "./urlState";

// ?tab=worldview|topics picks the page; bench params imply the Topics tab (where the drawer lives).
type Tab = "2d" | "streams";
const TAB_IDS: Record<string, Tab> = { worldview: "2d", topics: "streams" };
const initialTab = (): Tab => TAB_IDS[getParam("tab") ?? ""] ?? (hasBenchParams() ? "streams" : "2d");

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
  const [tab, setTabState] = useState<Tab>(initialTab);
  const setTab = (t: Tab) => {
    setTabState(t);
    setUrlParam("tab", t === "2d" ? null : "topics"); // worldview is the default → keep the URL clean
  };
  const [mediaMode, setMediaMode] = useState<MediaMode>("auto");
  const { gateway, setGateway } = useGateway();
  const [gwText, setGwText] = useState(gateway);
  const [gwRecent, setGwRecent] = useState(recentGateways);
  // Snap the field to the committed (normalized) value — a pasted http://… must not linger raw.
  useEffect(() => setGwText(gateway), [gateway]);
  const commitGw = (raw: string) => {
    setGateway(raw);
    // When raw normalizes to the CURRENT gateway, context state doesn't change and the
    // effect above won't fire — snap the visible text ourselves.
    setGwText(normalizeGateway(raw) || gateway);
  };
  // Escape blurs, and the blur handler would commit the text Escape meant to discard —
  // state updates land after the synchronous blur, so a ref carries the intent.
  const gwCancel = useRef(false);

  return (
    <NetemProvider>
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
          <TopbarNetem />
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
            list="gw-recent"
            onChange={(e) => setGwText(e.target.value)}
            onFocus={(e) => {
              e.currentTarget.select(); // address-bar ergonomics: focus = replace
              setGwRecent(recentGateways());
            }}
            onKeyDown={(e) => {
              if (e.key === "Enter") {
                commitGw(gwText);
                (e.target as HTMLInputElement).blur();
              }
              if (e.key === "Escape") {
                gwCancel.current = true;
                (e.target as HTMLInputElement).blur();
              }
            }}
            onBlur={() => {
              if (gwCancel.current) {
                gwCancel.current = false;
                setGwText(gateway);
                return;
              }
              commitGw(gwText);
            }}
            placeholder="localhost:8080"
            title="gateway server (host:port; pasted URLs are normalized) — reconnects on change; recent gateways remembered"
            spellCheck={false}
          />
          <datalist id="gw-recent">
            {gwRecent.map((g) => <option key={g} value={g} />)}
          </datalist>
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

      {/* The drawer stays mounted across tabs (hidden via CSS) so a running matrix sweep
          survives a glance at WorldView; StreamsTab stays conditional so its live
          subscriptions still stop when the tab is left. */}
      <div className="streams-full" style={tab === "streams" ? undefined : { display: "none" }}>
        {tab === "streams" && <StreamsTab />}
        <BenchDrawer />
      </div>
    </div>
    </NetemProvider>
  );
}
