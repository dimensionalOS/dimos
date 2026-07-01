// StatsBar — live per-topic hz / bandwidth / latency. Demonstrates the SDK's
// observability surface (each row keeps its topic subscribed while visible).
import { useTopics, useTopicStats } from "../dimos";

function Row({ topic }: { topic: string }) {
  const s = useTopicStats(topic);
  return (
    <tr>
      <td className="mono">{topic}</td>
      <td>{s?.hz ?? 0}</td>
      <td>{s ? (s.bytesPerSec / 1000).toFixed(1) : "0"}</td>
      <td>{s?.lastLatencyMs != null ? s.lastLatencyMs.toFixed(1) : "–"}</td>
    </tr>
  );
}

export function StatsBar() {
  const topics = useTopics();
  return (
    <div className="panel">
      <div className="panel-title">Stats (live)</div>
      <table className="stats">
        <thead>
          <tr>
            <th>topic</th>
            <th>hz</th>
            <th>kB/s</th>
            <th>lat ms</th>
          </tr>
        </thead>
        <tbody>
          {topics.map((t) => <Row key={t.topic} topic={t.topic} />)}
        </tbody>
      </table>
    </div>
  );
}
