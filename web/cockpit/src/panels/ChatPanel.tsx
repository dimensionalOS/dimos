// Agent chat panel: the humancli conversation as a panel. Messages arrive as
// chat.json.v1 frames (one array of {role, text, ts, tool?} lines per
// LangChain message) on a reliable channel; the transcript lives in
// chatTranscript.ts for the session's life, not the panel's. Typed text goes
// out through session.publish on the tx channel and renders when the agent
// echoes it back on the message channel, exactly like humancli; the
// composer's push-to-talk mic (ChatMic) joins the same conversation through
// the audio channel. Nothing typed or held here can drive the robot: the
// teleop pad listens on its own subtree and disarms when focus leaves it.

import { type FormEvent, useEffect, useRef, useState, useSyncExternalStore } from "react";
import type { PanelSpec } from "@dimos/shared";
import type { ChannelStore, Session } from "@dimos/sdk";
import { useStatus, useStoreChannel } from "@dimos/sdk/react";
import { PanelFrame } from "../layout/PanelFrame.tsx";
import { ChatMic } from "./ChatMic.tsx";
import { type ChatLine, chatTranscript } from "./chatTranscript.ts";
import type { PanelProps } from "./registry.tsx";
import styles from "./ChatPanel.module.css";

const KNOWN_ROLES = new Set(["human", "ai", "system"]);

function LineView({ line }: { line: ChatLine }) {
  if (line.tool !== undefined) {
    // Condensed one-liners, humancli style: the agent's call, then results
    // and progress under the tool's name.
    const text = line.role === "ai" ? `▶ ${line.text}` : `↳ ${line.tool}: ${line.text}`;
    return (
      <div
        className={styles.toolLine}
        title={line.text}
        data-role={line.role}
        data-tool={line.tool}
      >
        {text}
      </div>
    );
  }
  const cls = line.role === "human"
    ? styles.human
    : line.role === "ai"
    ? styles.ai
    : line.role === "system"
    ? styles.system
    : styles.other;
  return (
    <div className={cls} data-role={line.role}>
      {!KNOWN_ROLES.has(line.role) && <span className={styles.roleTag}>[{line.role}]</span>}
      <span className={styles.text}>{line.text}</span>
      <span className={styles.time}>{new Date(line.ts * 1000).toLocaleTimeString()}</span>
    </div>
  );
}

export function ChatPanel({ spec, store, session }: PanelProps) {
  if (spec.channels.length < 4 || session === undefined) {
    return (
      <PanelFrame spec={spec}>
        <span className={styles.hint}>chat panel {spec.id}: no send path bound</span>
      </PanelFrame>
    );
  }
  return <Conversation spec={spec} store={store} session={session} />;
}

function Conversation({ spec, store, session }: {
  spec: PanelSpec;
  store: ChannelStore;
  session: Session;
}) {
  const [inputCh, messagesCh, idleCh, audioCh] = spec.channels;
  const transcript = chatTranscript(store, messagesCh);
  const lines = useSyncExternalStore(transcript.subscribe, transcript.getSnapshot);
  const idle = useStoreChannel(store, idleCh).slot?.value;
  const connected = useStatus(session).transport.phase === "connected";
  const [draft, setDraft] = useState("");
  const [pending, setPending] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const listRef = useRef<HTMLDivElement | null>(null);
  // Autoscroll follows new lines unless the user scrolled up to read back.
  const pinned = useRef(true);

  useEffect(() => {
    const list = listRef.current;
    if (list !== null && pinned.current) list.scrollTop = list.scrollHeight;
  }, [lines]);

  const onScroll = (): void => {
    const list = listRef.current;
    if (list !== null) {
      pinned.current = list.scrollHeight - list.scrollTop - list.clientHeight < 8;
    }
  };

  const onSubmit = (e: FormEvent): void => {
    e.preventDefault();
    const text = draft.trim();
    if (text === "" || pending || !connected) return;
    setDraft("");
    setError(null);
    setPending(true);
    session
      .publish(inputCh, text)
      .catch((err: unknown) => {
        // A PublishError message already reads "<code>: <reason>".
        setError(`send failed: ${err instanceof Error ? err.message : String(err)}`);
        // Give a failed message back unless the user already typed on.
        setDraft((current) => (current === "" ? text : current));
      })
      .finally(() => setPending(false));
  };

  const thinking = pending || idle === false;
  const badge = thinking
    ? (
      <span className={styles.thinking} data-testid={`chat-${messagesCh}-thinking`}>
        thinking...
      </span>
    )
    : undefined;
  return (
    <PanelFrame spec={spec} badge={badge}>
      <div className={styles.chat} data-testid={`chat-${messagesCh}`}>
        <div className={styles.lines} ref={listRef} onScroll={onScroll}>
          {lines.length === 0 && <span className={styles.hint}>no messages yet</span>}
          {lines.map((line, i) => <LineView key={i} line={line} />)}
        </div>
        {error !== null && (
          <div className={styles.error} role="alert">
            {error}
          </div>
        )}
        <form className={styles.composer} onSubmit={onSubmit}>
          <input
            className={styles.input}
            value={draft}
            placeholder={connected ? "message the agent" : "not connected"}
            disabled={!connected}
            aria-label={`chat input ${inputCh}`}
            data-testid={`chat-${inputCh}-input`}
            onChange={(e) => setDraft(e.target.value)}
          />
          <ChatMic session={session} ch={audioCh} connected={connected} onError={setError} />
          <button
            type="submit"
            className={styles.send}
            disabled={!connected || pending || draft.trim() === ""}
          >
            send
          </button>
        </form>
      </div>
    </PanelFrame>
  );
}
