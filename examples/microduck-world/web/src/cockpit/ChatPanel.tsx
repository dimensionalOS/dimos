// Adapted from DimOS 536679f, Apache-2.0. Application-specific presentation.
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
import { displayText, inlineTokens } from "./terminalText.ts";
import { useStatus } from "@dimos/sdk/react";
import { PanelFrame } from "./PanelFrame.tsx";
import { formatClock, type Row, rowPrefix } from "@dimos/cockpit/panels/chatFold.ts";
import { chatLogFor, type PendingLine } from "@dimos/cockpit/panels/chatLog.ts";
import styles from "./humancli.module.css";
import { useOptionalSlot } from "@dimos/cockpit/panels/hooks.ts";
import { paramChannel, readString } from "@dimos/cockpit/panels/panelParams.ts";
import type { PanelProps } from "@dimos/cockpit/panels/registry.tsx";
import { txReasonText } from "@dimos/cockpit/panels/txReason.ts";

/** Bridge cap for human_input text (relay_bridge_module._CHAT_IN_MAX_CHARS). */
const HUMANCLI_LOGO = "   \u2587\u2587\u2587\u2587\u2587\u2587\u2557 \u2587\u2587\u2557\u2587\u2587\u2587\u2557   \u2587\u2587\u2587\u2557\u2587\u2587\u2587\u2587\u2587\u2587\u2587\u2557\u2587\u2587\u2587\u2557   \u2587\u2587\u2557\u2587\u2587\u2587\u2587\u2587\u2587\u2587\u2557\u2587\u2587\u2557 \u2587\u2587\u2587\u2587\u2587\u2587\u2557 \u2587\u2587\u2587\u2557   \u2587\u2587\u2557 \u2587\u2587\u2587\u2587\u2587\u2557 \u2587\u2587\u2557\n   \u2587\u2587\u2554\u2550\u2550\u2587\u2587\u2557\u2587\u2587\u2551\u2587\u2587\u2587\u2587\u2557 \u2587\u2587\u2587\u2587\u2551\u2587\u2587\u2554\u2550\u2550\u2550\u2550\u255d\u2587\u2587\u2587\u2587\u2557  \u2587\u2587\u2551\u2587\u2587\u2554\u2550\u2550\u2550\u2550\u255d\u2587\u2587\u2551\u2587\u2587\u2554\u2550\u2550\u2550\u2587\u2587\u2557\u2587\u2587\u2587\u2587\u2557  \u2587\u2587\u2551\u2587\u2587\u2554\u2550\u2550\u2587\u2587\u2557\u2587\u2587\u2551\n   \u2587\u2587\u2551  \u2587\u2587\u2551\u2587\u2587\u2551\u2587\u2587\u2554\u2587\u2587\u2587\u2587\u2554\u2587\u2587\u2551\u2587\u2587\u2587\u2587\u2587\u2557  \u2587\u2587\u2554\u2587\u2587\u2557 \u2587\u2587\u2551\u2587\u2587\u2587\u2587\u2587\u2587\u2587\u2557\u2587\u2587\u2551\u2587\u2587\u2551   \u2587\u2587\u2551\u2587\u2587\u2554\u2587\u2587\u2557 \u2587\u2587\u2551\u2587\u2587\u2587\u2587\u2587\u2587\u2587\u2551\u2587\u2587\u2551\n   \u2587\u2587\u2551  \u2587\u2587\u2551\u2587\u2587\u2551\u2587\u2587\u2551\u255a\u2587\u2587\u2554\u255d\u2587\u2587\u2551\u2587\u2587\u2554\u2550\u2550\u255d  \u2587\u2587\u2551\u255a\u2587\u2587\u2557\u2587\u2587\u2551\u255a\u2550\u2550\u2550\u2550\u2587\u2587\u2551\u2587\u2587\u2551\u2587\u2587\u2551   \u2587\u2587\u2551\u2587\u2587\u2551\u255a\u2587\u2587\u2557\u2587\u2587\u2551\u2587\u2587\u2554\u2550\u2550\u2587\u2587\u2551\u2587\u2587\u2551\n   \u2587\u2587\u2587\u2587\u2587\u2587\u2554\u255d\u2587\u2587\u2551\u2587\u2587\u2551 \u255a\u2550\u255d \u2587\u2587\u2551\u2587\u2587\u2587\u2587\u2587\u2587\u2587\u2557\u2587\u2587\u2551 \u255a\u2587\u2587\u2587\u2587\u2551\u2587\u2587\u2587\u2587\u2587\u2587\u2587\u2551\u2587\u2587\u2551\u255a\u2587\u2587\u2587\u2587\u2587\u2587\u2554\u255d\u2587\u2587\u2551 \u255a\u2587\u2587\u2587\u2587\u2551\u2587\u2587\u2551  \u2587\u2587\u2551\u2587\u2587\u2587\u2587\u2587\u2587\u2587\u2557\n   \u255a\u2550\u2550\u2550\u2550\u2550\u255d \u255a\u2550\u255d\u255a\u2550\u255d     \u255a\u2550\u255d\u255a\u2550\u2550\u2550\u2550\u2550\u2550\u255d\u255a\u2550\u255d  \u255a\u2550\u2550\u2550\u255d\u255a\u2550\u2550\u2550\u2550\u2550\u2550\u255d\u255a\u2550\u255d \u255a\u2550\u2550\u2550\u2550\u2550\u255d \u255a\u2550\u255d  \u255a\u2550\u2550\u2550\u255d\u255a\u2550\u255d  \u255a\u2550\u255d\u255a\u2550\u2550\u2550\u2550\u2550\u2550\u255d";

export const CHAT_INPUT_MAX_CHARS = 900;
export const AGENT_MODE_NOTICE = "Switch to Agent mode to talk to the duck";
export const THINKING_TEXT = "◌ agent thinking";
/** An empty transcript means nothing has been said yet, not that the agent is
 * still coming up - "waiting for the agent..." read as a stuck spinner. */
export const EMPTY_TEXT = "Ask the duck something";
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
      <PanelFrame spec={{ ...spec, title: "humancli" }}>
        <span className={styles.hint}>chat panel {spec.id}: no channel bound</span>
      </PanelFrame>
    );
  }
  return <ChatView spec={spec} store={store} teleop={teleop} chans={chans} />;
}

function ChatView(props: PanelProps & { chans: ChatChannels }) {
  return props.teleop ? <ConnectedChat {...props} teleop={props.teleop} /> : <ChatContent {...props} />;
}
function ConnectedChat(props: PanelProps & { chans: ChatChannels; teleop: NonNullable<PanelProps["teleop"]> }) {
  const status = useStatus(props.teleop);
  const control = status.manifest?.panels.find(p => p.kind === "control");
  const command = control ? paramChannel(control, "command", 3) : undefined;
  return <ChatContent {...props} command={command} />;
}
function ChatContent({ spec, store, teleop, chans, command }: PanelProps & { chans: ChatChannels; command?: string }) {
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

  const inputRef = useRef<HTMLTextAreaElement>(null);
  const [editing, setEditing] = useState(false);
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

  const canCompose = teleop !== undefined && chans.input !== undefined && spec.params.read_only !== true;
  const canSend = teleop !== undefined && chans.input !== undefined && !wrongMode && spec.params.read_only !== true;

  const transmit = (text: string): boolean => {
    if (!canSend || teleop === undefined || chans.input === undefined) {
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
      if (canSend) send();
    }
  };
  const onKeyUp = (e: KeyboardEvent<HTMLTextAreaElement>): void => {
    e.stopPropagation();
  };


  const selectAgent = (target: EventTarget) => {
    if (!canCompose || (target as HTMLElement).closest("button,select,[data-panel-action]")) return;
    if (wrongMode && !editing && teleop && command) {
      const result = teleop.tx(command, { name: "set_mode", args: { mode: "agent" } });
      setError(result.ok ? null : txReasonText(result.reason));
      if (!result.ok) return;
    }
    setEditing(true);
  };
  return (
    <div className="mw-chat-shell" onPointerDownCapture={e => selectAgent(e.target)}
      onClick={e => { if (!(e.target as HTMLElement).closest("button,select,[data-panel-action]")) inputRef.current?.focus({ preventScroll: true }); }}
      onBlur={e => { if (!e.currentTarget.contains(e.relatedTarget as Node | null)) setEditing(false); }}>
    <PanelFrame spec={{ ...spec, title: "humancli" }}>
      <div className={styles.chat} data-testid={`chat-${chans.chat}`}>
        <div
          ref={listRef}
          className={styles.transcript}
          data-testid={`chat-${chans.chat}-transcript`}
          onScroll={onScroll}
        >
          <svg className={styles.ascii} viewBox={`0 0 ${Math.max(...HUMANCLI_LOGO.split("\n").map(line => line.length)) * 6} ${HUMANCLI_LOGO.split("\n").length * 12}`} role="img" aria-label="DIMENSIONAL" preserveAspectRatio="xMinYMin meet">
            {HUMANCLI_LOGO.split("\n").map((line, i) => <text key={i} x="0" y={i * 12 + 10} fontFamily="Menlo, Consolas, monospace" fontSize="10" textLength={line.length * 6} lengthAdjust="spacingAndGlyphs" xmlSpace="preserve">{line}</text>)}
          </svg>
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
                      disabled={!canSend}
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
            <span className={styles.hint}>{EMPTY_TEXT}</span>
          )}
        </div>
        <div className={styles.compose}>
          <textarea
            ref={inputRef}
            onFocus={e => selectAgent(e.target)}
            className={styles.input}
            data-testid={`chat-${chans.chat}-input`}
            aria-label="message to the agent"
            placeholder={wrongMode ? editing ? "Type now; Agent mode is connecting…" : "Click to talk to the agent" : canSend ? "Enter sends, Shift+Enter for a new line" : "Agent input unavailable"}
            rows={2}
            value={draft}
            disabled={!canCompose}
            readOnly={wrongMode && !editing}
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
            Enter ↵
          </button>
        </div>
        {wrongMode && <div className={styles.modeNotice} role="status">{editing ? "Switching to Agent mode… You can type now." : "Teleop mode · click here to talk to the agent"}</div>}
        {error !== null && (
          <div className={styles.error} role="alert" data-testid={`chat-${chans.chat}-error`}>
            {error}
          </div>
        )}
      </div>
    </PanelFrame>
    </div>
  );
}

function TranscriptRow({ row }: { row: Row }) {
  const text = row.kind === "agent" ? displayText(row.text) : row.text;
  if (row.kind === "stream_mark" || !text) return null;
  return (
    <div className={styles.row} data-kind={row.kind}>
      <span className={styles.prefix}>{rowPrefix(formatClock(row.t), row.sender)}</span>
      <span className={styles.text}>{inlineTokens(text).map((part, i) => part.kind === "strong" ? <strong key={i}>{part.text}</strong> : part.kind === "code" ? <code key={i}>{part.text}</code> : part.text)}</span>
    </div>
  );
}
