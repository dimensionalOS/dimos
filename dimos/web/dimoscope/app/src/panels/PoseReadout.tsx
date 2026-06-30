import { useTopicLatest, useTopics } from "@dimos/react";

export function PoseReadout({ topic }: { topic?: string }) {
  const topics = useTopics();
  const poseTopic = topic ?? topics.find((t) => t.type === "geometry_msgs.PoseStamped")?.topic ??
    "/odom";
  const { data, meta } = useTopicLatest<any>(poseTopic, { maxHz: 15 });
  const p = data?.pose?.position;
  const o = data?.pose?.orientation;
  const yaw = o
    ? (Math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z)) * 180) / Math.PI
    : 0;
  const f = (n?: number) => (typeof n === "number" ? n.toFixed(2) : "–");
  return (
    <div className="panel">
      <div className="panel-title">Pose · {poseTopic}</div>
      <div className="readout">
        <div>
          <span>x</span>
          {f(p?.x)} m
        </div>
        <div>
          <span>y</span>
          {f(p?.y)} m
        </div>
        <div>
          <span>yaw</span>
          {p ? yaw.toFixed(0) : "–"}°
        </div>
        <div>
          <span>lat</span>
          {meta?.latencyMs != null ? meta.latencyMs.toFixed(1) : "–"} ms
        </div>
      </div>
    </div>
  );
}
