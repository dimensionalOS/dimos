// StreamsTab — subscribe to any discovered topic (click a chip, type a name, or "all discovered")
// and watch its messages stream in with per-topic QoS controls. The live companion to the Bench
// tab's quantitative sweep. Purely discovery-driven, so it reflects whatever source is running.
import { useState } from "react";
import { useTopics, useTopicStats } from "../../dimos";
import { StreamCard } from "./StreamCard";

// A discovered topic as a toggle chip — click to enable/disable its subscription. Mirrors the
// LayerChip pattern in WorldView: useTopicStats is a passive read, so an off chip costs nothing
// on the wire and its dot/hz decay to 0 until a StreamCard actually subscribes the topic.
function TopicChip({ topic, type, on, onToggle, index }: {
  topic: string;
  type: string;
  on: boolean;
  onToggle: () => void;
  index: number;
}) {
  const stats = useTopicStats(topic);
  const live = on && !!stats && stats.hz > 0;
  const dot = live ? "var(--ok)" : on ? "var(--muted)" : "transparent";
  return (
    <button
      type="button"
      className={`topic-chip${on ? " on" : ""}`}
      onClick={onToggle}
      title={`${topic}${type ? ` · ${type}` : ""} — click to ${on ? "unsubscribe" : "subscribe"}`}
      style={{ animationDelay: `${Math.min(index, 24) * 18}ms` }}
    >
      <span
        className={`topic-chip-dot${live ? " live" : ""}`}
        style={{ background: dot, borderColor: on ? "transparent" : "var(--line-hi)" }}
      />
      <span className="topic-chip-name">{topic}</span>
      {live && <span className="topic-chip-hz">{stats!.hz}hz</span>}
    </button>
  );
}

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
  const toggle = (t: string) => (subs.includes(t) ? remove(t) : add([t]));

  // one chip per discovered topic ∪ subscribed (so a name-typed pre-arm still shows), sorted by name.
  const chips = [...new Set([...topics.map((t) => t.topic), ...subs])].sort();

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
            placeholder="/topic…  subscribe by name"
            value={text}
            onChange={(e) => setText(e.target.value)}
          />
          <datalist id="stream-topic-names">
            {topics.map((t) => <option key={t.topic} value={t.topic} />)}
          </datalist>
          <button type="submit" className="tab">+ subscribe</button>
        </form>
        <div className="bench-row" style={{ marginTop: 8 }}>
          <button type="button" className="tab" onClick={() => add(topics.map((t) => t.topic))}>
            all discovered
          </button>
          {subs.length > 0 && (
            <button type="button" className="tab" onClick={() => setSubs([])}>clear</button>
          )}
          <span className="muted small">
            {subs.length} / {chips.length} subscribed
          </span>
        </div>
        {chips.length > 0 && (
          <div className="topic-chips">
            {chips.map((t, i) => (
              <TopicChip
                key={t}
                topic={t}
                type={typeOf(t)}
                on={subs.includes(t)}
                onToggle={() => toggle(t)}
                index={i}
              />
            ))}
          </div>
        )}
      </div>

      {subs.length === 0
        ? (
          <div className="muted small" style={{ padding: "8px 2px" }}>
            No streams yet — click a topic above, or hit <b>all discovered</b>. (Topics appear
            once a source is running and the gateway discovers them.)
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
