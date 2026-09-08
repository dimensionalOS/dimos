// Pure transcript reducer for the chat panel: a port of the humancli
// rendering rules (dimos/cli/human/humancli.py: receive_msg, _add_message,
// _write_tool_call/_write_tool_result, _split_tool_message) onto the
// chat.json.v1 frames the relay bridge emits. One frame in, zero or more
// display rows out; the reducer has no clock and no I/O so it can be tested
// message by message.
//
// Rules reproduced:
// - prefix ` HH:MM:SS {sender:>8} │ ` (21 chars) then the stripped content;
//   continuation lines align under the content (the panel draws the bar);
// - SystemMessage -> sender "system", truncated to 1000 chars;
// - AIMessage: content -> sender "agent" (or, while a tool-stream update is
//   the reply target, an annotation of that stream); each tool call ->
//   `▶ name(compact-json-args)` as sender "tool", anchored by its call id;
//   nothing at all and no reply target -> dim `<no response>`;
// - ToolMessage -> `↳ content` spliced right under the matching call (the
//   anchor is consumed, so a second result for the same call appends);
// - HumanMessage `[tool:NAME] text` -> a tool-stream row under NAME which
//   also becomes the reply target; a typed line clears the target and
//   renders as sender "human".
// The reply target is derived from the rows (the latest human/stream row)
// so it survives the module-level transcript exactly like the rows do.

export const TOOL_CALL_MARKER = "▶";
export const TOOL_RESULT_MARKER = "↳";
export const NO_RESPONSE = "<no response>";
/** Transcript cap: the oldest rows fall off (bridge replay is 200 deep). */
export const MAX_ROWS = 2000;
/** Width of the humancli prefix up to and including the bar and its space. */
export const PREFIX_WIDTH = 21;
const TOOL_MSG_PREFIX = "[tool:";
const SYSTEM_MAX_CHARS = 1000;

export type ChatRole = "human" | "ai" | "tool" | "system";

export interface ChatToolCall {
  id: string | null;
  name: string;
  args: unknown;
}

/** One chat.json.v1 frame, validated (see readChatMsg). */
export interface ChatMsg {
  /** Bridge-process monotonic entry number; replays repeat it. */
  n: number;
  role: ChatRole;
  content: string;
  name: string | null;
  tool_calls: ChatToolCall[];
  tool_call_id: string | null;
  id: string | null;
  /** Source time in seconds (frame header ts: original arrival, even on replay). */
  t: number;
}

export type RowKind =
  | "human"
  | "agent"
  | "tool"
  | "tool_result"
  | "system"
  | "dim"
  | "stream"
  | "stream_reply"
  /** Zero-height marker: a `[tool:NAME]` update without text still sets the
   * reply target (and must dedupe on replay), so it leaves a hidden row. */
  | "stream_mark";

export interface Row {
  /** Stable React key: `${n}/${index within the message}`. */
  key: string;
  n: number;
  /** Seconds (source clock). */
  t: number;
  /** Right-aligned to 8 columns by the renderer ("" for the spinner). */
  sender: string;
  kind: RowKind;
  text: string;
  /** Tool-call rows: the call id a result may splice under. */
  callId?: string;
  /** Tool-result rows: the call they answered (a consumed anchor). */
  forCallId?: string;
}

/** Validate one decoded chat.json.v1 value; `ts` (frame header seconds)
 * overrides the body's `t`, which is the encode time and so is wrong for
 * replays. Junk yields null and is skipped, never a crash. */
export function readChatMsg(v: unknown, ts?: number): ChatMsg | null {
  if (typeof v !== "object" || v === null) return null;
  const o = v as Record<string, unknown>;
  if (typeof o.n !== "number" || !Number.isFinite(o.n)) return null;
  const role = o.role;
  if (role !== "human" && role !== "ai" && role !== "tool" && role !== "system") return null;
  const toolCalls: ChatToolCall[] = [];
  if (Array.isArray(o.tool_calls)) {
    for (const tc of o.tool_calls) {
      if (typeof tc !== "object" || tc === null) continue;
      const { id, name, args } = tc as Record<string, unknown>;
      toolCalls.push({
        id: typeof id === "string" ? id : null,
        name: typeof name === "string" ? name : "unknown",
        args: args === undefined ? {} : args,
      });
    }
  }
  let t: number;
  if (typeof ts === "number" && Number.isFinite(ts)) t = ts;
  else if (typeof o.t === "number" && Number.isFinite(o.t)) t = o.t;
  else t = Date.now() / 1000;
  return {
    n: o.n,
    role,
    content: typeof o.content === "string" ? o.content : "",
    name: typeof o.name === "string" ? o.name : null,
    tool_calls: toolCalls,
    tool_call_id: typeof o.tool_call_id === "string" ? o.tool_call_id : null,
    id: typeof o.id === "string" ? o.id : null,
    t,
  };
}

/** humancli _split_tool_message: `[tool:NAME] text` -> [NAME, text]. */
export function splitToolMessage(content: string): [string, string] | null {
  if (!content.startsWith(TOOL_MSG_PREFIX)) return null;
  const end = content.indexOf("]");
  if (end === -1) return null;
  return [content.slice(TOOL_MSG_PREFIX.length, end), content.slice(end + 1).trimStart()];
}

/** humancli _format_tool_call: `▶ name({"k":"v"})` with compact JSON. */
export function formatToolCall(call: ChatToolCall): string {
  let args: string;
  try {
    args = JSON.stringify(call.args ?? {}) ?? "null";
  } catch {
    args = String(call.args);
  }
  return `${TOOL_CALL_MARKER} ${call.name}(${args})`;
}

/** dimos.utils.generic.truncate_display_string with an explicit max. */
export function truncateDisplay(s: string, max: number): string {
  return s.length <= max ? s : s.slice(0, max) + "...(truncated)...";
}

/** The humancli _reply_target: the tool whose box takes the agent's next
 * content, set by the latest tool-stream update and cleared by a typed line. */
export function replyTarget(rows: readonly Row[]): string | null {
  for (let i = rows.length - 1; i >= 0; i--) {
    const row = rows[i];
    if (row.kind === "human") return null;
    if (row.kind === "stream" || row.kind === "stream_mark") return row.sender;
  }
  return null;
}

/** True when a frame with this entry number already produced rows (bridge
 * replay after a resubscribe, or a duplicate delivery). */
export function hasEntry(rows: readonly Row[], n: number): boolean {
  for (let i = rows.length - 1; i >= 0; i--) if (rows[i].n === n) return true;
  return false;
}

/**
 * Fold one message into the transcript. Returns the same array when nothing
 * changed (duplicate `n`, or a silent agent step), a new array otherwise.
 */
export function foldChat(rows: readonly Row[], msg: ChatMsg): Row[] {
  if (hasEntry(rows, msg.n)) return rows as Row[];
  const out = rows.slice();
  let index = 0;
  const row = (kind: RowKind, sender: string, text: string): Row => ({
    key: `${msg.n}/${index++}`,
    n: msg.n,
    t: msg.t,
    sender,
    kind,
    text: text.trim(),
  });

  switch (msg.role) {
    case "system":
      out.push(row("system", "system", truncateDisplay(msg.content, SYSTEM_MAX_CHARS)));
      break;
    case "ai": {
      const target = replyTarget(rows);
      const content = msg.content;
      if (content !== "" && target !== null) {
        out.push(row("stream_reply", "agent", content));
      } else if (content !== "") {
        out.push(row("agent", "agent", content));
      }
      for (const call of msg.tool_calls) {
        const r = row("tool", "tool", formatToolCall(call));
        if (call.id !== null && call.id !== "") r.callId = call.id;
        out.push(r);
      }
      if (content === "" && msg.tool_calls.length === 0 && target === null) {
        out.push(row("dim", "agent", NO_RESPONSE));
      }
      break;
    }
    case "tool": {
      const r = row("tool_result", "tool", `${TOOL_RESULT_MARKER} ${msg.content}`);
      const callId = msg.tool_call_id;
      let at = out.length;
      if (callId !== null && callId !== "") {
        // The anchor is consumed by its first result: a later result for the
        // same call (a lookout continuation) appends like humancli's.
        const consumed = out.some((x) => x.forCallId === callId);
        if (!consumed) {
          for (let i = out.length - 1; i >= 0; i--) {
            if (out[i].callId === callId) {
              at = i + 1;
              r.forCallId = callId;
              break;
            }
          }
        }
      }
      out.splice(at, 0, r);
      break;
    }
    case "human": {
      const parsed = splitToolMessage(msg.content);
      if (parsed !== null) {
        const [name, text] = parsed;
        out.push(text !== "" ? row("stream", name, text) : row("stream_mark", name, ""));
      } else {
        out.push(row("human", "human", msg.content));
      }
      break;
    }
  }

  if (out.length === rows.length) return rows as Row[];
  if (out.length > MAX_ROWS) out.splice(0, out.length - MAX_ROWS);
  return out;
}

function pad2(n: number): string {
  return n < 10 ? `0${n}` : `${n}`;
}

/** Local wall clock HH:MM:SS for a source time in seconds. */
export function formatClock(tSec: number): string {
  const d = new Date(tSec * 1000);
  return `${pad2(d.getHours())}:${pad2(d.getMinutes())}:${pad2(d.getSeconds())}`;
}

/** The 19 columns before the bar: ` HH:MM:SS {sender:>8} `. */
export function rowPrefix(clock: string, sender: string): string {
  return ` ${clock} ${sender.padStart(8)} `;
}
