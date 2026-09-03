// Agent chat panel: the humancli transcript in a browser. Rows come from the
// per-channel ChatLog (direct store path, so lines land as they arrive);
// the idle flag and mode ride the UI tick. Typed lines go out through the
// session's generic tx on the panel's input channel and stay "pending" until
// the agent loop echoes the HumanMessage back on the chat channel.
//
// Key events never leave the textarea (stopPropagation on keydown/keyup):
// WASD typed here must not reach the teleop pad, and no global shortcuts
// are registered by this panel.

import {
  type KeyboardEvent,
  type UIEvent,
  useLayoutEffect,
  useMemo,
  useRef,
  useState,
  useSyncExternalStore,
} from "react";
import { PanelFrame } from "../layout/PanelFrame.tsx";
import { formatClock, type Row, rowPrefix } from "./chatFold.ts";
import { chatLogFor, type PendingLine } from "./chatLog.ts";
import styles from "./ChatPanel.module.css";
import { useOptionalSlot } from "./hooks.ts";
import { paramChannel, readString } from "./panelParams.ts";
import type { PanelProps } from "./registry.tsx";
import { txReasonText } from "./txReason.ts";

/** Bridge cap for human_input text (relay_bridge_module._CHAT_IN_MAX_CHARS). */
export const CHAT_INPUT_MAX_CHARS = 900;
export const AGENT_MODE_NOTICE = "Switch to Agent mode to talk to the duck";
export const THINKING_TEXT = "◌ agent thinking";
/** Scroll slack under which the transcript still counts as "at the bottom". */
const STICK_SLACK_PX = 8;

export interface ChatChannels {
  chat: string;
  idle: string | undefined;
  mode: string | undefined;
  input: string | undefined;
}

export function chatChannels(spec: PanelProps["spec"]): ChatChannels | null {
  const chat = paramChannel(spec, "chat", 0);
  if (chat === undefined) return null;
  return {
    chat,
    idle: paramChannel(spec, "idle", 1),
    mode: paramChannel(spec, "mode", 2),
    input: paramChannel(spec, "input", 3),
  };
}

function readFlag(v: unknown): boolean | null {
  if (typeof v !== "object" || v === null) return null;
  const value = (v as Record<string, unknown>).value;
  return typeof value === "boolean" ? value : null;
}

function readMode(v: unknown): string | null {
  if (typeof v !== "object" || v === null) return null;
  return readString((v as Record<string, unknown>).mode);
}

export function ChatPanel({ spec, store, teleop }: PanelProps) {
  const chans = chatChannels(spec);
  if (chans === null) {
    return (
      <PanelFrame spec={spec}>
        <span className={styles.hint}>chat panel {spec.id}: no channel bound</span>
      </PanelFrame>
    );
  }
  return <ChatView spec={spec} store={store} teleop={teleop} chans={chans} />;
}

function ChatView({ spec, store, teleop, chans }: PanelProps & { chans: ChatChannels }) {
  const log = chatLogFor(store, chans.chat);
  const { rows, pending } = useSyncExternalStore(log.subscribe, log.getSnapshot);
  const idleSlot = useOptionalSlot(store, chans.idle);
  const modeSlot = useOptionalSlot(store, chans.mode);
  const idle = readFlag(idleSlot?.value);
  const mode = readMode(modeSlot?.value);
  const thinking = idle === false;
  // humancli stamps the spinner with the time it appeared.
  const thinkingSince = useMemo(() => (thinking ? Date.now() / 1000 : 0), [thinking]);
  const wrongMode = mode !== null && mode !== "agent";

  const [draft, setDraft] = useState("");
  const [error, setError] = useState<string | null>(null);
  const listRef = useRef<HTMLDivElement | null>(null);
  const stick = useRef(true);

  useLayoutEffect(() => {
    const el = listRef.current;
    if (el !== null && stick.current) el.scrollTop = el.scrollHeight;
  }, [rows, pending, thinking]);

  const onScroll = (e: UIEvent<HTMLDivElement>): void => {
    const el = e.currentTarget;
    stick.current = el.scrollHeight - el.scrollTop - el.clientHeight <= STICK_SLACK_PX;
  };

  const canSend = teleop !== undefined && chans.input !== undefined;

  const transmit = (text: string): boolean => {
    if (teleop === undefined || chans.input === undefined) {
      setError("no send path bound");
      return false;
    }
    const result = teleop.tx(chans.input, { text });
    if (!result.ok) {
      setError(txReasonText(result.reason));
      return false;
    }
    setError(null);
    return true;
  };

  const send = (): void => {
    const text = draft.trim();
    if (text === "") return;
    if (text.length > CHAT_INPUT_MAX_CHARS) {
      setError(`message too long (max ${CHAT_INPUT_MAX_CHARS} chars)`);
      return;
    }
    if (transmit(text)) {
      log.addPending(text);
      setDraft("");
      stick.current = true;
    }
  };

  const retry = (line: PendingLine): void => {
    if (transmit(line.text)) log.resent(line.key);
  };

  const onKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>): void => {
    e.stopPropagation();
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      send();
    }
  };
  const onKeyUp = (e: KeyboardEvent<HTMLTextAreaElement>): void => {
    e.stopPropagation();
  };

  if (wrongMode) {
    return (
      <PanelFrame spec={spec}>
        <div className={styles.notice} data-testid={`chat-${chans.chat}-notice`}>
          {AGENT_MODE_NOTICE}
        </div>
      </PanelFrame>
    );
  }

  return (
    <PanelFrame spec={spec}>
      <div className={styles.chat} data-testid={`chat-${chans.chat}`}>
        <div
          ref={listRef}
          className={styles.transcript}
          data-testid={`chat-${chans.chat}-transcript`}
          onScroll={onScroll}
        >
          {rows.map((row) => <TranscriptRow key={row.key} row={row} />)}
          {pending.map((line) => (
            <div
              key={line.key}
              className={styles.row}
              data-kind="human"
              data-status={line.status}
              data-testid="chat-pending"
            >
              <span className={styles.prefix}>
                {rowPrefix(formatClock(line.sentAt / 1000), "human")}
              </span>
              <span className={styles.text}>
                {line.text}
                {line.status === "failed" && (
                  <span className={styles.failed}>
                    not delivered ·
                    <button
                      type="button"
                      className={styles.retry}
                      onMouseDown={(e) => e.preventDefault()}
                      onClick={() => retry(line)}
                    >
                      retry
                    </button>
                  </span>
                )}
              </span>
            </div>
          ))}
          {thinking && (
            <div className={styles.row} data-kind="thinking" data-testid="chat-thinking">
              <span className={styles.prefix}>{rowPrefix(formatClock(thinkingSince), "")}</span>
              <span className={styles.text}>{THINKING_TEXT}</span>
            </div>
          )}
          {rows.length === 0 && pending.length === 0 && !thinking && (
            <span className={styles.hint}>waiting for the agent...</span>
          )}
        </div>
        <div className={styles.compose}>
          <textarea
            className={styles.input}
            data-testid={`chat-${chans.chat}-input`}
            aria-label="message to the agent"
            placeholder={canSend ? "Enter sends, Shift+Enter for a new line" : "no send path bound"}
            rows={2}
            value={draft}
            disabled={!canSend}
            maxLength={CHAT_INPUT_MAX_CHARS * 2}
            onChange={(e) => setDraft(e.target.value)}
            onKeyDown={onKeyDown}
            onKeyUp={onKeyUp}
          />
          <button
            type="button"
            className={styles.send}
            data-testid={`chat-${chans.chat}-send`}
            disabled={!canSend || draft.trim() === ""}
            onMouseDown={(e) => e.preventDefault()}
            onClick={send}
          >
            send
          </button>
        </div>
        {error !== null && (
          <div className={styles.error} role="alert" data-testid={`chat-${chans.chat}-error`}>
            {error}
          </div>
        )}
      </div>
    </PanelFrame>
  );
}

function TranscriptRow({ row }: { row: Row }) {
  if (row.kind === "stream_mark") return null;
  return (
    <div className={styles.row} data-kind={row.kind}>
      <span className={styles.prefix}>{rowPrefix(formatClock(row.t), row.sender)}</span>
      <span className={styles.text}>{row.text}</span>
    </div>
  );
}
