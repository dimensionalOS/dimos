// App shell: connection status + a topic-discovery sidebar + the panel area.
// Click a discovered topic to open a type-aware panel for it (registry).
import { useState } from "react";
import { useTopics } from "./useBus";
import { WorldView } from "./widgets/WorldView";
import { TeleopPad } from "./widgets/TeleopPad";
import { widgetForType } from "./widgets/registry";

export function App() {
  const { topics, status } = useTopics();
  const [selected, setSelected] = useState<string | null>(null);
  const list = [...topics.entries()].sort();

  const Panel = selected ? widgetForType(topics.get(selected) ?? "?") : null;

  return (
    <div className="layout">
      <header className="topbar">
        <b>dimoscope</b>
        <span className={`status status-${status}`}>● {status}</span>
        <span className="muted">a live viewer for the dimos LCM bus</span>
      </header>

      <aside className="sidebar">
        <div className="sidebar-title">Topics ({list.length})</div>
        {list.length === 0 && <div className="muted small">discovering… is the bridge + robot running?</div>}
        {list.map(([topic, type]) => (
          <button
            key={topic}
            className={`topic ${selected === topic ? "topic-sel" : ""}`}
            onClick={() => setSelected(topic)}
          >
            <div className="topic-name">{topic}</div>
            <div className="topic-type">{type}</div>
          </button>
        ))}
      </aside>

      <main className="main">
        <WorldView />
        <div className="side-col">
          <TeleopPad />
          {Panel && selected ? (
            <Panel topic={selected} />
          ) : (
            <div className="panel">
              <div className="panel-title">Inspector</div>
              <div className="muted small">select a topic on the left to inspect it</div>
            </div>
          )}
        </div>
      </main>
    </div>
  );
}
