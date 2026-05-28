# Scaffold reference — original /webapp (teammate's App Router stub)

Preserved for merging. Goldie (the current /webapp) replaced this stub. The stub
was wired to the **monorepo DimOS backend** (Tailscale, port 8443), which uses a
DIFFERENT contract than Goldie currently targets — keep this to merge later:

- SSE stream key: `agent_state` (structured JSON), NOT `agent_responses` (plain text).
- Auth: `Authorization: Bearer <token>` on fetch + `?token=<token>` on the SSE URL.
- Structured state: phase, current_skill{name,args,state}, last_observation,
  last_narration ("robot said"), awaiting_user ("robot is asking"), intent.
- Input tags: <user_speech>, <user_reply> (when awaiting_user set), <user_command> (e.g. "stop").

TODO to run Goldie against the monorepo backend: re-point `src/lib/dimos.ts` + the
agent feed to `agent_state` + token, add the awaiting_user → <user_reply> flow, and
speak `last_narration` / `awaiting_user` via TTS. See `webapp/AGENTS.md`.

## app/page.tsx
```tsx
"use client";

import { useCallback, useEffect, useRef, useState } from "react";

const API = process.env.NEXT_PUBLIC_DIMOS_API ?? "";
const TOKEN = process.env.NEXT_PUBLIC_DIMOS_TOKEN ?? "";

type AgentState = {
  ts?: number;
  intent?: string;
  phase?: string;
  current_skill?: { name?: string; args?: unknown; state?: string } | null;
  last_observation?: string;
  last_narration?: string;
  awaiting_user?: string | null;
};

export default function Page() {
  const [state, setState] = useState<AgentState>({});
  const [recording, setRecording] = useState(false);
  const [connected, setConnected] = useState(false);
  const [lastError, setLastError] = useState<string>("");
  const [partialTranscript, setPartialTranscript] = useState<string>("");

  const recognitionRef = useRef<any>(null);
  const finalTranscriptRef = useRef<string>("");

  useEffect(() => {
    if (!API) return;
    const url = `${API}/text_stream/agent_state?token=${encodeURIComponent(TOKEN)}`;
    const es = new EventSource(url);
    es.onopen = () => setConnected(true);
    es.onmessage = (e) => {
      try {
        setState(JSON.parse(e.data));
      } catch {
        // ignore non-JSON keepalives
      }
    };
    es.onerror = () => setConnected(false);
    return () => es.close();
  }, []);

  const sendText = useCallback(async (text: string, tag: "user_speech" | "user_reply" | "user_command") => {
    setLastError("");
    try {
      const fd = new FormData();
      fd.append("query", `<${tag}>${text}</${tag}>`);
      const r = await fetch(`${API}/submit_query`, {
        method: "POST",
        headers: { Authorization: `Bearer ${TOKEN}` },
        body: fd,
      });
      if (!r.ok) setLastError(`submit_query ${r.status}`);
    } catch (e) {
      setLastError(String(e));
    }
  }, []);

  const startRecording = useCallback(async () => {
    setLastError("");
    setPartialTranscript("");
    finalTranscriptRef.current = "";

    // Browser-side STT via Web Speech API (free, fast, runs on the phone).
    // iOS Safari exposes it as webkitSpeechRecognition; Chrome/Edge also have it.
    const SR: any =
      (typeof window !== "undefined" && (window as any).SpeechRecognition) ||
      (typeof window !== "undefined" && (window as any).webkitSpeechRecognition);

    if (!SR) {
      setLastError("speech recognition not supported in this browser");
      return;
    }

    try {
      const rec = new SR();
      rec.continuous = false;
      rec.interimResults = true;
      rec.lang = navigator.language || "en-US";
      rec.maxAlternatives = 1;

      rec.onresult = (event: any) => {
        let final = "";
        let interim = "";
        for (let i = event.resultIndex; i < event.results.length; i++) {
          const r = event.results[i];
          if (r.isFinal) final += r[0].transcript;
          else interim += r[0].transcript;
        }
        if (final) finalTranscriptRef.current += final;
        setPartialTranscript(finalTranscriptRef.current + interim);
      };

      rec.onerror = (event: any) => {
        setLastError(`stt: ${event.error || "unknown"}`);
        setRecording(false);
      };

      rec.onend = () => {
        setRecording(false);
        const text = finalTranscriptRef.current.trim();
        setPartialTranscript("");
        if (text) {
          const tag = state.awaiting_user ? "user_reply" : "user_speech";
          sendText(text, tag);
        }
      };

      rec.start();
      recognitionRef.current = rec;
      setRecording(true);
    } catch (e) {
      setLastError(`mic: ${String(e)}`);
    }
  }, [sendText, state.awaiting_user]);

  const stopRecording = useCallback(() => {
    try {
      recognitionRef.current?.stop();
    } catch {
      // already stopped
    }
  }, []);

  const stopButton = useCallback(() => {
    sendText("stop", "user_command");
  }, [sendText]);

  const [textDraft, setTextDraft] = useState("");
  const sendDraft = useCallback(() => {
    if (!textDraft.trim()) return;
    const tag = state.awaiting_user ? "user_reply" : "user_speech";
    sendText(textDraft.trim(), tag);
    setTextDraft("");
  }, [textDraft, sendText, state.awaiting_user]);

  return (
    <main style={{ minHeight: "100vh", padding: 24, display: "flex", flexDirection: "column", gap: 16 }}>
      <header style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
        <h1 style={{ fontSize: 18, fontWeight: 600 }}>Dimos Guide</h1>
        <span style={{ fontSize: 12, color: connected ? "#4ade80" : "#f87171" }}>
          {connected ? "● connected" : "○ reconnecting"}
        </span>
      </header>

      <section style={{ display: "flex", flexDirection: "column", alignItems: "center", gap: 12, marginTop: 24 }}>
        <button
          onTouchStart={(e) => {
            e.preventDefault();
            startRecording();
          }}
          onTouchEnd={(e) => {
            e.preventDefault();
            stopRecording();
          }}
          onMouseDown={startRecording}
          onMouseUp={stopRecording}
          onMouseLeave={() => recording && stopRecording()}
          style={{
            width: 220,
            height: 220,
            borderRadius: "50%",
            border: "none",
            background: recording ? "#dc2626" : state.awaiting_user ? "#f59e0b" : "#1e3a8a",
            color: "white",
            fontSize: 18,
            fontWeight: 600,
            touchAction: "none",
            transition: "background 0.1s",
          }}
          aria-label={recording ? "Recording — release to send" : "Hold to speak"}
        >
          {recording ? "Listening…" : state.awaiting_user ? "Tap to reply" : "Hold to speak"}
        </button>

        <button
          onClick={stopButton}
          style={{
            marginTop: 8,
            padding: "12px 24px",
            borderRadius: 8,
            border: "1px solid #6b7280",
            background: "transparent",
            color: "#e5e5e5",
          }}
        >
          Stop
        </button>

        <div style={{ display: "flex", gap: 8, width: "100%", marginTop: 16 }}>
          <input
            type="text"
            inputMode="text"
            autoComplete="off"
            autoCapitalize="off"
            placeholder="Or type a query…"
            value={textDraft}
            onChange={(e) => setTextDraft(e.target.value)}
            onKeyDown={(e) => { if (e.key === "Enter") sendDraft(); }}
            style={{
              flex: 1,
              padding: "12px 14px",
              borderRadius: 8,
              border: "1px solid #374151",
              background: "#111827",
              color: "#e5e5e5",
              fontSize: 16,
              minWidth: 0,
            }}
          />
          <button
            onClick={sendDraft}
            disabled={!textDraft.trim()}
            style={{
              padding: "12px 18px",
              borderRadius: 8,
              border: "1px solid #2563eb",
              background: textDraft.trim() ? "#2563eb" : "#1e3a8a",
              color: "white",
              opacity: textDraft.trim() ? 1 : 0.6,
            }}
          >
            Send
          </button>
        </div>
      </section>

      {partialTranscript ? (
        <section style={{ padding: 12, background: "#1f1f1f", borderRadius: 8, border: "1px solid #2563eb" }}>
          <div style={{ fontSize: 11, color: "#60a5fa", textTransform: "uppercase", marginBottom: 4 }}>
            Hearing
          </div>
          <div style={{ fontStyle: "italic" }}>{partialTranscript}</div>
        </section>
      ) : null}

      {state.awaiting_user ? (
        <section style={{ padding: 12, background: "#1f1f1f", borderRadius: 8, border: "1px solid #f59e0b" }}>
          <div style={{ fontSize: 11, color: "#f59e0b", textTransform: "uppercase", marginBottom: 4 }}>
            Robot is asking
          </div>
          <div>{state.awaiting_user}</div>
        </section>
      ) : null}

      {state.last_narration ? (
        <section style={{ padding: 12, background: "#1f1f1f", borderRadius: 8 }}>
          <div style={{ fontSize: 11, color: "#9ca3af", textTransform: "uppercase", marginBottom: 4 }}>
            Robot said
          </div>
          <div>{state.last_narration}</div>
        </section>
      ) : null}

      <section style={{ marginTop: "auto", fontSize: 11, color: "#6b7280" }}>
        <div>phase: {state.phase ?? "—"}</div>
        <div>skill: {state.current_skill?.name ?? "—"} ({state.current_skill?.state ?? "—"})</div>
        <div>sees: {state.last_observation ?? "—"}</div>
        {lastError && <div style={{ color: "#f87171", marginTop: 6 }}>error: {lastError}</div>}
      </section>

      <details style={{ fontSize: 11, color: "#6b7280" }}>
        <summary>Raw state JSON</summary>
        <pre>{JSON.stringify(state, null, 2)}</pre>
      </details>
    </main>
  );
}
```

## app/layout.tsx
```tsx
import "./globals.css";
import type { Metadata, Viewport } from "next";

export const metadata: Metadata = {
  title: "Dimos Guide",
  description: "Voice-driven robot guide for low-vision users",
};

export const viewport: Viewport = {
  width: "device-width",
  initialScale: 1,
  maximumScale: 1,
  userScalable: false,
  themeColor: "#0a0a0a",
};

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
```

## app/globals.css
```css
* { box-sizing: border-box; margin: 0; padding: 0; }
html, body {
  height: 100%;
  background: #0a0a0a;
  color: #e5e5e5;
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
  -webkit-tap-highlight-color: transparent;
  overscroll-behavior: none;
}
button { font: inherit; -webkit-user-select: none; user-select: none; }
pre { white-space: pre-wrap; word-break: break-word; }
```

## .env.local.example
```
NEXT_PUBLIC_DIMOS_API=https://dog.yourname.dev
NEXT_PUBLIC_DIMOS_TOKEN=replace-with-DIMOS_API_TOKEN-from-vps
```
