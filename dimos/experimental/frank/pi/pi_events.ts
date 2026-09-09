// Consume inbox events only while this Pi process owns the active turn.
import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";

export default function (pi: ExtensionAPI) {
  let timer: ReturnType<typeof setTimeout> | undefined;
  let closed = false;
  let pending: AbortController | undefined;
  const base = (process.env.FRANK_URL || "http://127.0.0.1:7790").replace(/\/$/, "");
  const headers: Record<string, string> = {};
  if (process.env.FRANK_AGENT_TOKEN) headers.Authorization = `Bearer ${process.env.FRANK_AGENT_TOKEN}`;
  async function poll() {
    pending = new AbortController();
    const timeout = setTimeout(() => pending?.abort(), 5000);
    try {
      const response = await fetch(`${base}/agent/events?wait=0`, { headers, signal: pending.signal });
      if (!response.ok) throw new Error(`Inbox HTTP ${response.status}`);
      const event = await response.json();
      if (!closed && event.type !== "none") {
        pi.sendMessage({
          customType: "frank-event",
          content: `FRANK event received during this turn:\n${JSON.stringify(event)}\nHandle this now. Tool events are robot feedback, not a person speaking. A found event ends that search. Close wake tasks when handled.`,
          display: true,
        }, { deliverAs: "steer", triggerTurn: true });
      }
    } catch (error) {
      if (!closed) console.error(`[frank inbox listener] ${String(error)}`);
    } finally {
      clearTimeout(timeout);
      if (!closed) timer = setTimeout(poll, 1000);
    }
  }
  pi.on("session_start", async () => { timer = setTimeout(poll, 1000); });
  pi.on("session_shutdown", async () => {
    closed = true;
    clearTimeout(timer);
    pending?.abort();
  });
}
