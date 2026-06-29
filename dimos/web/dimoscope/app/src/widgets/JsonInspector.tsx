// Universal fallback widget: pretty-print the latest decoded message.
// Long arrays are truncated and the message classes' methods are hidden.
import { useTopic } from "../useBus";

function clean(_k: string, v: any) {
  if (typeof v === "function") return undefined;
  if (typeof v === "bigint") return v.toString();
  if (Array.isArray(v) && v.length > 16) return [...v.slice(0, 16), `… +${v.length - 16} more`];
  return v;
}

export function JsonInspector({ topic }: { topic: string }) {
  const msg = useTopic(topic);
  return (
    <div className="panel">
      <div className="panel-title">JSON · {topic}</div>
      <pre className="json">
        {msg ? JSON.stringify(msg.data, clean, 2) : "waiting for a message…"}
      </pre>
    </div>
  );
}
