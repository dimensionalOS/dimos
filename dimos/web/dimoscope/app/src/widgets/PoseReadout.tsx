// A tiny type-aware widget for PoseStamped: shows x / y / yaw as text.
// Demonstrates that the registry can pick a purpose-built widget per message type.
import { useTopic } from "../useBus";

function quatToYaw(q: { x: number; y: number; z: number; w: number }) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

export function PoseReadout({ topic }: { topic: string }) {
  const msg = useTopic(topic);
  const p = msg?.data?.pose;
  return (
    <div className="panel">
      <div className="panel-title">Pose · {topic}</div>
      {p ? (
        <div className="readout">
          <div><span>x</span>{p.position.x.toFixed(3)} m</div>
          <div><span>y</span>{p.position.y.toFixed(3)} m</div>
          <div><span>yaw</span>{((quatToYaw(p.orientation) * 180) / Math.PI).toFixed(1)}°</div>
        </div>
      ) : (
        <div className="readout">waiting…</div>
      )}
    </div>
  );
}
