import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import type { FrameHeader } from "@dimos/shared";
import { ChannelStore } from "@dimos/sdk";
import {
  type ChatMsg,
  foldChat,
  formatClock,
  MAX_ROWS,
  NO_RESPONSE,
  PREFIX_WIDTH,
  readChatMsg,
  replyTarget,
  type Row,
  rowPrefix,
  splitToolMessage,
} from "./chatFold.ts";
import { ChatLog, chatLogFor, RETRY_AFTER_MS } from "./chatLog.ts";

let counter = 0;

function msg(partial: Partial<ChatMsg> & { role: ChatMsg["role"] }): ChatMsg {
  counter += 1;
  return {
    n: counter,
    content: "",
    name: null,
    tool_calls: [],
    tool_call_id: null,
    id: null,
    t: 1_700_000_000 + counter,
    ...partial,
  };
}

function fold(...msgs: ChatMsg[]): Row[] {
  let rows: Row[] = [];
  for (const m of msgs) rows = foldChat(rows, m);
  return rows;
}

const texts = (rows: readonly Row[]) => rows.map((r) => r.text);

beforeEach(() => {
  counter = 0;
});

describe("foldChat", () => {
  it("appends human, agent and system rows in arrival order with humancli senders", () => {
    const rows = fold(
      msg({ role: "human", content: "  hello duck  " }),
      msg({ role: "ai", content: "hi there" }),
      msg({ role: "system", content: "note" }),
    );
    expect(rows.map((r) => [r.kind, r.sender, r.text])).toEqual([
      ["human", "human", "hello duck"],
      ["agent", "agent", "hi there"],
      ["system", "system", "note"],
    ]);
    expect(rows.map((r) => r.n)).toEqual([1, 2, 3]);
  });

  it("dedupes on n: a replayed entry changes nothing, even out of order", () => {
    const a = msg({ role: "human", content: "one" });
    const b = msg({ role: "ai", content: "two" });
    const rows = fold(a, b);
    expect(foldChat(rows, a)).toBe(rows);
    expect(foldChat(rows, { ...b, content: "different text, same n" })).toBe(rows);
    expect(texts(rows)).toEqual(["one", "two"]);
  });

  it("renders tool calls as ▶ name(compact json) and splices ↳ results under their call", () => {
    const call = msg({
      role: "ai",
      content: "looking",
      tool_calls: [
        { id: "c1", name: "go_to", args: { room: "kitchen", speed: 0.5 } },
        { id: "c2", name: "look", args: {} },
      ],
    });
    const human = msg({ role: "human", content: "hurry" });
    const result2 = msg({ role: "tool", content: "saw a cup", tool_call_id: "c2" });
    const result1 = msg({ role: "tool", content: "arrived", tool_call_id: "c1" });
    const rows = fold(call, human, result2, result1);
    expect(texts(rows)).toEqual([
      "looking",
      '▶ go_to({"room":"kitchen","speed":0.5})',
      "↳ arrived",
      "▶ look({})",
      "↳ saw a cup",
      "hurry",
    ]);
    expect(rows[1].callId).toBe("c1");
    expect(rows[2].forCallId).toBe("c1");
    expect(rows.filter((r) => r.kind === "tool").every((r) => r.sender === "tool")).toBe(true);
  });

  it("consumes the anchor: a second result for the same call appends, an unknown id appends", () => {
    const rows = fold(
      msg({ role: "ai", tool_calls: [{ id: "c1", name: "lookout", args: {} }] }),
      msg({ role: "tool", content: "first", tool_call_id: "c1" }),
      msg({ role: "human", content: "ok" }),
      msg({ role: "tool", content: "second", tool_call_id: "c1" }),
      msg({ role: "tool", content: "orphan", tool_call_id: "nope" }),
    );
    expect(texts(rows)).toEqual(["▶ lookout({})", "↳ first", "ok", "↳ second", "↳ orphan"]);
  });

  it("routes [tool:NAME] human messages to a stream row and the next agent content to it", () => {
    expect(splitToolMessage("[tool:lookout]   cup ahead")).toEqual(["lookout", "cup ahead"]);
    expect(splitToolMessage("[tool:broken")).toBeNull();
    expect(splitToolMessage("plain")).toBeNull();

    const rows = fold(
      msg({ role: "human", content: "[tool:lookout] cup ahead" }),
      msg({ role: "ai", content: "noted, a cup" }),
      msg({ role: "ai" }), // silent step after a stream update: nothing
      msg({ role: "human", content: "what do you see?" }),
      msg({ role: "ai" }), // silent step after a typed line: <no response>
    );
    expect(rows.map((r) => [r.kind, r.sender, r.text])).toEqual([
      ["stream", "lookout", "cup ahead"],
      ["stream_reply", "agent", "noted, a cup"],
      ["human", "human", "what do you see?"],
      ["dim", "agent", NO_RESPONSE],
    ]);
  });

  it("keeps the reply target on an empty [tool:NAME] update via a hidden marker row", () => {
    const rows = fold(msg({ role: "human", content: "[tool:lookout]" }));
    expect(rows).toHaveLength(1);
    expect(rows[0].kind).toBe("stream_mark");
    expect(replyTarget(rows)).toBe("lookout");
    const more = foldChat(rows, msg({ role: "ai" }));
    expect(more).toBe(rows); // silent step stays silent
    expect(replyTarget(fold(msg({ role: "human", content: "typed" })))).toBeNull();
  });

  it("shows <no response> only for an empty agent step with no tool calls", () => {
    const rows = fold(
      msg({ role: "ai" }),
      msg({ role: "ai", tool_calls: [{ id: null, name: "stop", args: {} }] }),
    );
    expect(texts(rows)).toEqual([NO_RESPONSE, "▶ stop({})"]);
    expect(rows[1].callId).toBeUndefined();
  });

  it("truncates system content at 1000 chars like truncate_display_string", () => {
    const rows = fold(msg({ role: "system", content: "x".repeat(1200) }));
    expect(rows[0].text).toBe("x".repeat(1000) + "...(truncated)...");
  });

  it("caps the transcript at MAX_ROWS, dropping the oldest", () => {
    let rows: Row[] = [];
    for (let i = 0; i < MAX_ROWS + 5; i++) {
      rows = foldChat(rows, msg({ role: "human", content: `m${i}` }));
    }
    expect(rows).toHaveLength(MAX_ROWS);
    expect(rows[0].text).toBe("m5");
  });

  it("formats the 21-column humancli prefix with the sender right-aligned to 8", () => {
    const prefix = rowPrefix("12:34:56", "agent");
    expect(prefix).toBe(" 12:34:56    agent ");
    expect(prefix.length + "│ ".length).toBe(PREFIX_WIDTH);
    expect(rowPrefix("12:34:56", "")).toBe(" 12:34:56          ");
    const d = new Date(2024, 0, 1, 7, 5, 9);
    expect(formatClock(d.getTime() / 1000)).toBe("07:05:09");
  });
});

describe("readChatMsg", () => {
  it("validates the chat.json.v1 shape and prefers the frame ts over the body t", () => {
    const value = {
      n: 7,
      role: "ai",
      content: "hi",
      name: null,
      tool_calls: [{ id: "a", name: "x", args: { k: 1 } }, "junk", { name: 3 }],
      tool_call_id: null,
      id: "run-1",
      t: 99,
    };
    const m = readChatMsg(value, 123.5);
    expect(m).not.toBeNull();
    expect(m?.t).toBe(123.5);
    expect(m?.tool_calls).toEqual([
      { id: "a", name: "x", args: { k: 1 } },
      { id: null, name: "unknown", args: {} },
    ]);
    expect(readChatMsg(value)?.t).toBe(99);
    expect(readChatMsg({ role: "ai" })).toBeNull();
    expect(readChatMsg({ n: 1, role: "robot" })).toBeNull();
    expect(readChatMsg("nope")).toBeNull();
  });
});

describe("ChatLog", () => {
  const CH = "agent";
  let now: number;
  let store: ChannelStore;
  let seq: number;

  function header(): FrameHeader {
    seq += 1;
    return { ch: CH, seq, ts: now / 1000, delivery: "reliable" };
  }

  function ingest(value: Record<string, unknown>): void {
    counter += 1;
    store.ingest(CH, header(), { n: counter, ...value }, true);
  }

  beforeEach(() => {
    vi.useFakeTimers();
    now = 1_700_000_000_000;
    vi.setSystemTime(now);
    store = new ChannelStore(() => now);
    seq = 0;
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  it("folds frames as they arrive and settles a pending line on its echo", () => {
    const log = new ChatLog(store, CH, () => now);
    const seen: number[] = [];
    log.subscribe(() => seen.push(log.getSnapshot().rows.length));

    log.addPending("go to the kitchen");
    expect(log.getSnapshot().pending).toEqual([
      { key: "pending-1", text: "go to the kitchen", sentAt: now, status: "pending" },
    ]);

    ingest({ role: "ai", content: "ready" });
    expect(texts(log.getSnapshot().rows)).toEqual(["ready"]);
    expect(log.getSnapshot().pending).toHaveLength(1);

    ingest({ role: "human", content: "go to the kitchen " });
    expect(texts(log.getSnapshot().rows)).toEqual(["ready", "go to the kitchen"]);
    expect(log.getSnapshot().pending).toEqual([]);
    expect(seen).toEqual([0, 1, 2]);
  });

  it("marks a line not delivered after RETRY_AFTER_MS and re-arms on resend", () => {
    const log = new ChatLog(store, CH, () => now);
    log.addPending("hello");
    now += RETRY_AFTER_MS - 1;
    vi.advanceTimersByTime(RETRY_AFTER_MS - 1);
    expect(log.getSnapshot().pending[0].status).toBe("pending");
    now += 1;
    vi.advanceTimersByTime(1);
    expect(log.getSnapshot().pending[0].status).toBe("failed");

    log.resent("pending-1");
    expect(log.getSnapshot().pending[0]).toMatchObject({ status: "pending", sentAt: now });
    now += RETRY_AFTER_MS;
    vi.advanceTimersByTime(RETRY_AFTER_MS);
    expect(log.getSnapshot().pending[0].status).toBe("failed");

    // A late echo still settles a failed line; dismiss drops it outright.
    log.addPending("second");
    ingest({ role: "human", content: "hello" });
    expect(log.getSnapshot().pending.map((p) => p.text)).toEqual(["second"]);
    log.dismiss("pending-2");
    expect(log.getSnapshot().pending).toEqual([]);
  });

  it("picks up a slot that predates it, ignores junk, and empties on store.reset()", () => {
    ingest({ role: "human", content: "early" });
    const log = new ChatLog(store, CH, () => now);
    expect(texts(log.getSnapshot().rows)).toEqual(["early"]);
    store.ingest(CH, header(), { role: "ai" }, true); // no n: skipped
    expect(log.getSnapshot().rows).toHaveLength(1);

    store.reset();
    expect(log.getSnapshot().rows).toEqual([]);
    // A restarted producer reuses low entry numbers; they are not duplicates now.
    counter = 0;
    ingest({ role: "human", content: "fresh start" });
    expect(texts(log.getSnapshot().rows)).toEqual(["fresh start"]);
  });

  it("chatLogFor hands out one log per store and channel", () => {
    const a = chatLogFor(store, CH);
    expect(chatLogFor(store, CH)).toBe(a);
    expect(chatLogFor(store, "other")).not.toBe(a);
    expect(chatLogFor(new ChannelStore(), CH)).not.toBe(a);
  });
});
