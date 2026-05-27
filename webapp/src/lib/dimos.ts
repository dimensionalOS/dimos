import { fetchEventSource } from "@microsoft/fetch-event-source";
import type { MoveCommand } from "./types";

/**
 * Single client for the DimOS backend (WEBAPP-BRIEF §5).
 * Defaults to the local mock (`/api/mock`) so the UI runs with no backend;
 * point NEXT_PUBLIC_DIMOS_API at the real VPS to switch.
 *
 * NOTE (monorepo merge): this client currently targets the `agent_responses`
 * (plain-text) + ngrok-header contract. The monorepo DimOS backend instead
 * streams structured `agent_state` JSON and uses token auth (Bearer + ?token=).
 * To run against it: switch the stream key to `agent_state`, send the token,
 * and map last_narration/awaiting_user in the feed. See ../../SCAFFOLD-REFERENCE.md
 * and webapp/AGENTS.md ("Backend contracts").
 */
const API = (process.env.NEXT_PUBLIC_DIMOS_API ?? "/api/mock").replace(/\/+$/, "");
const TOKEN = process.env.NEXT_PUBLIC_DIMOS_TOKEN ?? "";

function headers(extra: Record<string, string> = {}): Record<string, string> {
  // ngrok-skip-browser-warning avoids the free-tier HTML interstitial that
  // otherwise breaks fetch/SSE through the tunnel (WEBAPP-INTEGRATION.md).
  const h: Record<string, string> = {
    "ngrok-skip-browser-warning": "true",
    ...extra,
  };
  if (TOKEN) h.Authorization = `Bearer ${TOKEN}`;
  return h;
}

/** Text query → agent. multipart/form-data, field `query` (NOT JSON). */
export async function submitQuery(query: string): Promise<void> {
  const fd = new FormData();
  fd.append("query", query);
  await fetch(`${API}/submit_query`, {
    method: "POST",
    body: fd,
    headers: headers(),
  });
}

/** Upload a recorded clip; field name MUST be `file`. Returns the transcript. */
export async function uploadAudio(
  blob: Blob,
  filename = "recording.mp4",
): Promise<string> {
  const fd = new FormData();
  fd.append("file", blob, filename);
  const res = await fetch(`${API}/upload_audio`, {
    method: "POST",
    body: fd,
    headers: headers(),
  });
  const data = await res.json();
  return (data?.text as string) ?? "";
}

export async function listStreams(): Promise<string[]> {
  const res = await fetch(`${API}/text_streams`, { headers: headers() });
  const data = await res.json();
  return data?.streams ?? [];
}

/** Direct sport command, bypassing the LLM. JSON body. */
export async function unitreeCommand(command: string): Promise<void> {
  await fetch(`${API}/unitree/command`, {
    method: "POST",
    headers: headers({ "Content-Type": "application/json" }),
    body: JSON.stringify({ command }),
  });
}

export async function getStatus(): Promise<{ connected: boolean }> {
  try {
    const res = await fetch(`${API}/unitree/status`, { headers: headers() });
    if (!res.ok) return { connected: false };
    const data = await res.json();
    // Real backend: {status:"online",...}; mock: {connected:true}. Accept both.
    return { connected: data?.status === "online" || data?.connected === true };
  } catch {
    return { connected: false };
  }
}

/** TBD: joystick drive — endpoint not yet defined in the brief; stubbed in the mock. */
export async function move(cmd: MoveCommand): Promise<void> {
  await fetch(`${API}/unitree/move`, {
    method: "POST",
    headers: headers({ "Content-Type": "application/json" }),
    body: JSON.stringify(cmd),
  });
}

/** TBD: interrupt the running agent — endpoint not yet defined; stubbed in the mock. */
export async function interrupt(): Promise<void> {
  await fetch(`${API}/interrupt`, { method: "POST", headers: headers() });
}

export type StreamStatus = "connecting" | "open" | "error";

/**
 * Subscribe to a named SSE stream. Calls `onData` with each frame's raw `data`
 * string (caller parses defensively). Returns an unsubscribe function.
 * Uses fetch-event-source so it works with auth headers in production.
 */
export function subscribeStream(
  key: string,
  onData: (data: string) => void,
  onStatus?: (s: StreamStatus) => void,
): () => void {
  const ctrl = new AbortController();
  onStatus?.("connecting");

  fetchEventSource(`${API}/text_stream/${key}`, {
    signal: ctrl.signal,
    headers: headers(),
    openWhenHidden: true,
    onopen: async () => {
      onStatus?.("open");
    },
    onmessage: (ev) => {
      if (ev.data) onData(ev.data);
    },
    onerror: () => {
      onStatus?.("error");
      // returning undefined lets fetch-event-source retry with backoff
    },
  }).catch(() => {
    // aborted on unsubscribe — ignore
  });

  return () => ctrl.abort();
}
