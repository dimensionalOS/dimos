import assert from "node:assert/strict";
import sharp from "sharp";
import { createHash } from "node:crypto";
import { mkdtemp, rm, writeFile } from "node:fs/promises";
import { join } from "node:path";
import { tmpdir } from "node:os";
import { setTimeout as delay } from "node:timers/promises";
import { renderLayoutFrame } from "@earendil-works/pi-tui/dist/layout.js";
import type { AssistantMessage } from "@earendil-works/pi-ai";
import { fauxAssistantMessage } from "@earendil-works/pi-ai";
import type { TranscriptItem } from "#dimos/agents/shared/protocol/conversation/state.ts";
import type { TimelineRow } from "#dimos/agents/shared/view-models/conversation/timeline.ts";
import { AssistantMessageComponent, UserMessageComponent, initTheme } from "@earendil-works/pi-coding-agent";
import {
  ProcessTerminal,
  ScrollView,
  TuiMainScreen,
  setCapabilities,
  setCellDimensions,
  visibleWidth,
} from "@earendil-works/pi-tui";
import { CONVERSATION_ACTIONS } from "#dimos/agents/shared/protocol/actions/conversation.ts";
import { EntryIdSchema } from "#dimos/agents/shared/protocol/args.ts";
import { RenderDetailsSchema } from "#dimos/agents/shared/protocol/media.ts";
import { TranscriptView, type TranscriptSource } from "#dimos/agents/frontends/tui/conversation/transcript.ts";
import { Value } from "typebox/value";
import { stripVTControlCharacters } from "node:util";
import { test } from "node:test";

const ENTRY = Value.Parse(EntryIdSchema, 1);
const REQUEST = Value.Parse(CONVERSATION_ACTIONS.send.args.properties.request, "11111111-1111-4111-8111-111111111111");
const ANSWER = fauxAssistantMessage("  Checking **the door**.\n\n- One\n- Two\n  ");

function itemRow(item: TranscriptItem): TimelineRow {
  return { kind: "item", key: `e${item.entry}`, item, outcome: null };
}

function view(rows: readonly TimelineRow[], openPanel: ConstructorParameters<typeof TranscriptView>[3] = () => {}) {
  initTheme("dark", false);
  const tui = new TuiMainScreen(new ProcessTerminal());
  tui.requestRender = () => {};
  let source: TranscriptSource = { rows, omitted: 0 };
  const transcript = new TranscriptView(
    tui,
    () => process.cwd(),
    () => source,
    openPanel,
  );
  return {
    transcript,
    update: (next: readonly TimelineRow[]) => {
      source = { rows: next, omitted: 0 };
    },
  };
}

function plain(lines: readonly string[]): string[] {
  return lines.map((line) => stripVTControlCharacters(line).trimEnd());
}

test("historical and native user messages keep Markdown, prompt boundaries and an unlabeled background", (t) => {
  const text = "3. **Open** the door\n4. Keep \\*literal\\*\n\n```py\nprint('hello')\n```";
  const { transcript, update } = view([]);
  t.after(() => transcript.dispose());
  const messages: TranscriptItem[] = [
    { kind: "user", entry: ENTRY, message: { role: "user", content: text, timestamp: 0 } },
    {
      kind: "input",
      entry: ENTRY,
      input: { source: { kind: "user", request: REQUEST }, text, change: null, render: null, limit: null },
      images: [],
    },
  ];
  for (const message of messages) {
    transcript.reset();
    update([itemRow(message)]);
    for (const width of [24, 80]) {
      const rendered = transcript.render(width);
      const expected = new UserMessageComponent(text, undefined, 3).render(width);
      assert.equal(plain(rendered)[0], "");
      assert.deepEqual(plain(rendered.slice(1)), plain(expected));
      for (const zone of ["A", "B", "C"]) {
        assert.equal(rendered.join("").split(`\x1b]133;${zone}\x07`).length, 2);
      }
      assert(rendered[1]?.startsWith("\x1b]133;A\x07"));
      assert(rendered.at(-1)?.startsWith("\x1b]133;B\x07\x1b]133;C\x07"));
      assert(rendered[2]?.includes("48;"), "the native user background remains visible");
      assert(rendered.every((line) => visibleWidth(line) <= width));
    }
  }
});

test("streaming text keeps completed assistant spacing through blank, growing and tool-only updates", (t) => {
  const streaming = (message: AssistantMessage): TimelineRow => ({ kind: "streaming", key: "streaming", message });
  const { transcript, update } = view([streaming({ ...ANSWER, content: [] })]);
  t.after(() => transcript.dispose());
  assert.deepEqual(transcript.render(24), []);
  for (const text of ["  First ", "  Checking **the door**.\n\n- One\n- Two\n  ", "\n  "]) {
    const message = { ...ANSWER, content: [{ type: "text", text }] } satisfies AssistantMessage;
    update([streaming(message)]);
    for (const width of [24, 80]) {
      assert.deepEqual(
        plain(transcript.render(width)),
        plain(new AssistantMessageComponent(message, true).render(width)),
      );
    }
  }
  update([
    streaming({
      ...ANSWER,
      content: [
        { type: "thinking", thinking: "private" },
        { type: "toolCall", id: "call", name: "inventory", arguments: {} },
      ],
    }),
  ]);
  assert.deepEqual(transcript.render(24), []);
  update([streaming(ANSWER)]);
  const before = plain(transcript.render(24));
  update([itemRow({ kind: "assistant", entry: ENTRY, message: ANSWER })]);
  assert.deepEqual(plain(transcript.render(24)), before);
});

test("running tools indent output and background hint, and still expand", (t) => {
  const row = {
    kind: "tool",
    key: "tcall",
    call: { type: "toolCall", id: "call", name: "inventory", arguments: {} },
    slot: { callId: "call", name: "inventory", status: "running", output: "first\nsecond" },
    backgroundable: true,
    since: null,
  } satisfies TimelineRow;
  const { transcript, update } = view([row]);
  t.after(() => transcript.dispose());
  const lines = plain(transcript.render(80));
  assert(lines.find((line) => line.includes("inventory"))?.startsWith("   ↳ "));
  assert(lines.find((line) => line.includes("Ctrl+B"))?.startsWith("     Ctrl+B"));
  transcript.setExpanded(true);
  assert(plain(transcript.render(80)).join("\n").includes("second"));
  update([{ ...row, backgroundable: false, slot: { ...row.slot, output: "updated" } }]);
  const updated = transcript.render(24);
  assert(!plain(updated).join("\n").includes("Ctrl+B"));
  assert(plain(updated).join("\n").includes("updated"));
  assert(updated.every((line) => visibleWidth(line) <= 24));
});

test("tool previews retain selected tab clicks and partially scrolled Kitty images", async (t) => {
  for (const key of ["TERM_PROGRAM", "WARP_SESSION_ID", "WARP_TERMINAL_SESSION_UUID"]) {
    const previous = process.env[key];
    delete process.env[key];
    t.after(() => {
      if (previous === undefined) delete process.env[key];
      else process.env[key] = previous;
    });
  }
  setCapabilities({ images: "kitty", trueColor: true, hyperlinks: false });
  setCellDimensions({ widthPx: 10, heightPx: 20 });
  const dir = await mkdtemp(join(tmpdir(), "transcript-image-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const png = await sharp({ create: { width: 400, height: 400, channels: 3, background: "red" } })
    .png()
    .toBuffer();
  const path = join(dir, "preview.png");
  await writeFile(path, png);
  const ref = { path, sha256: createHash("sha256").update(png).digest("hex") };
  const details = Value.Parse(RenderDetailsSchema, {
    title: "Views",
    listed: ["saved.png"],
    failed: [],
    views: ["A", "B"].map((label) => ({
      kind: "image",
      label,
      source: ref,
      preview: ref,
    })),
  });
  let opened = -1;
  const { transcript, update } = view([], (gallery, index) => {
    assert.equal(gallery, transcript.latestGallery);
    opened = index;
  });
  t.after(() => transcript.dispose());
  update([
    { kind: "item", key: "answer", item: { kind: "assistant", entry: ENTRY, message: ANSWER }, outcome: null },
    itemRow({
      kind: "tool_result",
      entry: ENTRY,
      result: {
        role: "toolResult",
        toolCallId: "call",
        toolName: "inventory",
        content: [{ type: "text", text: "Found views" }],
        details: { tool: "inventory", details: { render: details, limit: null } },
        isError: false,
        timestamp: 0,
      },
    }),
  ]);
  transcript.select(`e${ENTRY}`);
  const rendered = transcript.render(80);
  const lines = plain(rendered);
  const tabY = lines.findIndex((line) => line.includes("0 Overview"));
  assert(lines[tabY]?.startsWith("┃    0 Overview"));
  assert(lines.find((line) => line.includes("Also:"))?.startsWith("┃    Also:"));
  assert.equal(transcript.offsetOf(`e${ENTRY}`), new AssistantMessageComponent(ANSWER, true).render(80).length);
  const tabX = lines[tabY]?.indexOf("2 B");
  assert(tabX !== undefined && tabX >= 0);
  assert.equal(
    transcript.handleMouse({
      type: "press",
      button: "left",
      x: tabX,
      y: tabY,
      screenX: tabX,
      screenY: tabY,
      width: 80,
      height: rendered.length,
      shift: false,
      alt: false,
      ctrl: false,
    })?.handled,
    true,
  );
  assert.equal(opened, 2);
  assert(transcript.render(24).every((line) => visibleWidth(line) <= 24));
  transcript.select(null);
  let anchor = -1;
  const deadline = Date.now() + 3000;
  while (anchor < 0 && Date.now() < deadline) {
    anchor = transcript.render(80).findIndex((line) => line.includes("\x1b_G"));
    if (anchor < 0) await delay(10);
  }
  assert(anchor >= 0, "the real PNG preview renders");
  const scroll = new ScrollView(transcript, { follow: "none", scrollbar: "hidden" });
  renderLayoutFrame(scroll, 80, 5, () => {});
  scroll.scrollTo(anchor + 2);
  const frame = renderLayoutFrame(scroll, 80, 5, () => {});
  assert.equal(frame.lines.filter((line) => line.includes("\x1b_G")).length, 1, "partially visible preview remains");
  transcript.reset();
  assert.equal(transcript.latestGallery, null);
});
