import assert from "node:assert/strict";
import sharp from "sharp";
import type { Channel, MsgType, StreamRef } from "#dimos/agents/shared/protocol/streams.ts";
import type { FrameRecord, FrameSource } from "#dimos/agents/sdk/streams/base.ts";
import type { ModuleInstance, ModuleRow } from "#dimos/agents/shared/protocol/runtime/module.ts";
import type { SidebarSource } from "#dimos/agents/frontends/tui/sidebar/sidebar.ts";
import type { Terminal } from "@earendil-works/pi-tui/dist/terminal.js";
import { HStack, TuiAltScreen, setCapabilities, setCellDimensions } from "@earendil-works/pi-tui";
import { PromptEditor } from "#dimos/agents/frontends/tui/conversation/input.ts";
import { Sidebar } from "#dimos/agents/frontends/tui/sidebar/sidebar.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { initTheme } from "@earendil-works/pi-coding-agent";
import { setTimeout as delay } from "node:timers/promises";
import { test } from "node:test";

class MemoryTerminal implements Terminal {
  columns = 100;
  rows = 12;
  kittyProtocolActive = false;
  private onInput: (data: string) => void = () => {};
  start(onInput: (data: string) => void): void {
    this.onInput = onInput;
  }
  stop(): void {}
  async drainInput(): Promise<void> {}
  write(): void {}
  moveBy(): void {}
  hideCursor(): void {}
  showCursor(): void {}
  clearLine(): void {}
  clearFromCursor(): void {}
  clearScreen(): void {}
  setTitle(): void {}
  setProgress(): void {}
  setProgramStatus(): void {}
  input(data: string): void {
    this.onInput(data);
  }
}

class MemoryFrames implements FrameSource {
  wanted = new Set<Channel>();
  private listener: (record: FrameRecord) => void = () => {};
  want(channels: ReadonlySet<Channel>): void {
    this.wanted = new Set(channels);
  }
  onRecord(listener: (record: FrameRecord) => void): () => void {
    this.listener = listener;
    return () => {
      this.listener = () => {};
    };
  }
  close(): void {}
  emit(record: FrameRecord): void {
    this.listener(record);
  }
}

function ref(module: string, name: string, type: string): StreamRef {
  return {
    source: { module: module as ModuleInstance, name, direction: "out", type: type as MsgType },
    encoding:
      new Map([
        ["Image", "jpeg.v1"],
        ["PointCloud2", "voxels.zlib.v1"],
        ["OccupancyGrid", "costmap.zlib.v1"],
      ]).get(type) ?? "json.v1",
    channel: `${module}.${name}` as Channel,
  };
}

function moduleRow(instance: string): ModuleRow {
  return {
    name: "double" as ModuleRow["name"],
    class: "Double",
    owner: null,
    path: "double.py",
    lastError: null,
    state: "running",
    revision: 1,
    instance: instance as ModuleInstance,
  };
}

test("sidebar navigates measured picture rows and folds duplicate occurrences through the real TUI viewport", async (t) => {
  initTheme("dark", false);
  setCapabilities({ images: null, trueColor: false, hyperlinks: false });
  setCellDimensions({ widthPx: 10, heightPx: 20 });
  const terminal = new MemoryTerminal();
  const tui = new TuiAltScreen(terminal, false, undefined, { mouse: false });
  const frames = new MemoryFrames();
  const duplicate = ref("double", "shared", "Image");
  const system = [
    ref("robot", "camera_a", "Image"),
    ref("robot", "camera_b", "Image"),
    ref("robot", "camera_c", "Image"),
    ref("robot", "cloud", "PointCloud2"),
    ref("robot", "grid", "OccupancyGrid"),
  ];
  const source: SidebarSource = {
    streams: [{ ref: duplicate, via: { kind: "module", module: "double" as ModuleRow["name"] } }],
    modules: [moduleRow("one"), moduleRow("two")],
    jobs: [],
    transcript: { items: [], omitted: 0 },
    queue: { items: [] },
    system: { streams: system, modules: [] },
  };
  let current: SidebarSource | null = source;
  let changes = 0;
  const sidebar = new Sidebar(
    () => current,
    frames,
    () => {
      changes++;
      tui.requestRender();
    },
    () => terminal.rows,
  );
  const editor = new PromptEditor(tui, [], process.cwd());
  const layout = new HStack([
    { component: editor, basis: 0, grow: 1, minSize: 60 },
    {
      component: sidebar.column,
      basis: 40,
      visible: ({ width }) => {
        const visible = sidebar.isOpen && width >= 100;
        sidebar.setVisible(visible);
        if (!visible && sidebar.focused) tui.setFocus(editor);
        return visible;
      },
    },
  ]);
  tui.setLayoutRoot(layout);
  tui.setFocus(editor);
  tui.start();
  t.after(() => {
    sidebar.dispose();
    tui.stop({ preserveScreen: true });
  });
  editor.setValue("draft survives focus");
  tui.renderNow();
  assert.deepEqual(frames.wanted, new Set([duplicate.channel, ...system.map((item) => item.channel)]));
  assert.equal(sidebar.viewport.viewportHeight, 12);

  const beforePicture = sidebar.render(40).length;
  const jpeg = await sharp({ create: { width: 320, height: 240, channels: 3, background: "red" } })
    .jpeg()
    .toBuffer();
  frames.emit({ kind: "image", channel: system[0]!.channel, at: epochMs(Date.now()), width: 320, height: 240, jpeg });
  const deadline = Date.now() + 3000;
  while (sidebar.render(40).length <= beforePicture && Date.now() < deadline) await delay(10);
  assert(sidebar.render(40).length > beforePicture, "a real picture makes a multi-line row");

  tui.setFocus(sidebar);
  tui.renderNow();
  terminal.input("\x1b[D");
  tui.renderNow();
  assert(frames.wanted.has(duplicate.channel), "the other occurrence still demands its shared channel");
  terminal.input("\x1b[B");
  terminal.input("\x1b[D");
  tui.renderNow();
  assert(!frames.wanted.has(duplicate.channel), "folding both occurrences stops their shared frame demand");
  terminal.input("\x1b[B");
  tui.renderNow();
  const expandedHeight = sidebar.render(40).length;
  terminal.input("\x1b[D");
  tui.renderNow();
  assert(sidebar.render(40).length < expandedHeight, "folding hides the prepared picture");
  terminal.input("\x1b[C");
  tui.renderNow();
  assert.equal(sidebar.render(40).length, expandedHeight, "opening restores the last ready picture");
  for (let i = 0; i < 4; i++) terminal.input("\x1b[B");
  tui.renderNow();
  assert(sidebar.viewport.scrollTop > 0, "selection moves the native scroll offset beyond the screen");
  assert(
    tui.getScreenLines().some((line) => line.includes("grid")),
    "the grid is visible after several cameras",
  );
  terminal.rows = 8;
  tui.renderNow();
  assert.equal(sidebar.viewport.viewportHeight, 8);
  assert(
    tui.getScreenLines().some((line) => line.includes("grid")),
    "resize retains the selected row",
  );
  tui.setFocus(editor);
  tui.renderNow();
  assert.equal(editor.getValue(), "draft survives focus");
  sidebar.setVisible(false);
  assert.equal(frames.wanted.size, 0);
  current = null;
  sidebar.setVisible(true);
  tui.setFocus(sidebar);
  tui.renderNow();
  assert.equal(frames.wanted.size, 0, "a null source has no stale demand");
  const settled = changes;
  tui.renderNow();
  assert.equal(changes, settled, "focused empty content settles without another redraw request");
});
