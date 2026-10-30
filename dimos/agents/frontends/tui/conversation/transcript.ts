import type { AssistantMessage, ToolCall, UserMessage } from "@earendil-works/pi-ai";
import type { Component, TUI, TuiMouseEvent, TuiMouseEventResult } from "@earendil-works/pi-tui";
import type { DimosInput } from "../../../shared/protocol/conversation/input.ts";
import type { InputOutcome, ToolResult, TranscriptItem } from "../../../shared/protocol/conversation/state.ts";
import type { QueuedRow } from "../../../shared/view-models/conversation/queue.ts";
import type { RenderDetails } from "../../../shared/protocol/media.ts";
import type { TimelineRow } from "../../../shared/view-models/conversation/timeline.ts";
import type { ToolCard } from "../cards/base.ts";
import { AssistantMessageComponent, UserMessageComponent, getMarkdownTheme } from "@earendil-works/pi-coding-agent";
import { Container, Markdown, Spacer, Text } from "@earendil-works/pi-tui";
import { Gallery } from "../views/gallery.ts";
import { accent, amber, muted } from "../style.ts";
import { cardFor } from "../cards/choose.ts";
import { clean, cleanMultiline } from "../../../shared/view-models/text.ts";

/** What the chat shows now: the conversation's timeline and how many older items the transfer budget left out. */
export type TranscriptSource = { readonly rows: readonly TimelineRow[]; readonly omitted: number };

/** The component drawn for one timeline row, kept across renders while its key stays. */
type RowView = {
  readonly component: Component;
  update(row: TimelineRow): void;
  setExpanded(expanded: boolean): void;
  dispose(): void;
};

/** A line this terminal added itself (a command's answer, an /inspect rendering), placed after the newest item it saw. */
type Note = { readonly after: string | null; readonly component: Component; readonly gallery: Gallery | null };

function renderOf(result: ToolResult): RenderDetails | null {
  const details = result.details;
  switch (details.tool) {
    case "bash":
    case "inventory":
    case "stream_get":
    case "agent_module":
      return details.details.render;
    case "read":
    case "edit":
    case "write":
    case "job":
    case "background":
    case null:
      return null;
  }
}

/** A result's image blocks arrive without their bytes; the card shows each as its placeholder line. */
function withPlaceholders(result: ToolResult): ToolResult {
  if (!result.content.some((block) => block.type === "image")) return result;
  return {
    ...result,
    content: result.content.map((block) =>
      block.type === "image" ? { type: "text", text: `[image ${block.mimeType}]` } : block,
    ),
  };
}

/** The tool call a result answers, from the newest assistant item that made it. */
function callOf(rows: readonly TimelineRow[], result: ToolResult): ToolCall {
  for (let index = rows.length - 1; index >= 0; index--) {
    const row = rows[index];
    if (row === undefined || row.kind !== "item" || row.item.kind !== "assistant") continue;
    for (const block of row.item.message.content)
      if (block.type === "toolCall" && block.id === result.toolCallId) return block;
  }
  return { type: "toolCall", id: result.toolCallId, name: result.toolName, arguments: {} };
}

function userText(message: UserMessage): string {
  const content = message.content;
  return cleanMultiline(
    typeof content === "string"
      ? content
      : content.flatMap((block) => (block.type === "text" ? [block.text] : [])).join("\n"),
  );
}

function partialText(message: AssistantMessage): string {
  return cleanMultiline(message.content.flatMap((block) => (block.type === "text" ? [block.text] : [])).join(""));
}

function cleanAssistant(message: AssistantMessage): AssistantMessage {
  return {
    ...message,
    content: message.content.map((block) =>
      block.type === "text" ? { ...block, text: cleanMultiline(block.text) } : block,
    ),
  };
}

function firstLine(text: string): string {
  return clean(text.split("\n")[0] ?? "");
}

function line(text: string): Text {
  return new Text(text, 0, 0);
}

const FROM_LABELS: { readonly [F in QueuedRow["from"]]: string } = {
  you: "",
  module: "⚡",
  job: "⏰",
  background: "⇢",
  runtime: "↻",
};

/** A background call's result as the chat shows it; the model got all of it. */
const BACKGROUND_LINES = 3;

const SELECTED_GUTTER = "┃ ";

const BACKGROUND_ENDS: {
  readonly [E in Extract<DimosInput["source"], { readonly kind: "background" }>["end"]]: string;
} = { finished: "finished", failed: "failed", stopped: "was stopped", interrupted: "was interrupted" };

function queuedLine(row: QueuedRow): string {
  const label = row.from === "you" ? (row.mode === "steer" ? "steer" : "follow") : FROM_LABELS[row.from];
  return muted(`   queued ${label.padEnd(8)}`) + clean(row.text);
}

function outcomeKey(outcome: InputOutcome | null): string {
  if (outcome === null) return "";
  return outcome.status === "done" ? "done" : `unanswered ${outcome.reason}`;
}

/**
 * The chat: one pi-tui component per timeline row, reconciled by key when it renders, so a streamed partial updates its
 * one component and a new item adds one. Items keep their per-kind drawing (inputs by source, messages, tool cards with
 * their renderings); the rows are the state, so nothing here applies events or counts runs.
 */
export class TranscriptView implements Component {
  private readonly tui: TUI;
  private readonly cwd: () => string;
  private readonly source: () => TranscriptSource | null;
  private readonly openPanel: (gallery: Gallery, index: number) => void;
  private readonly views = new Map<string, RowView>();
  private notes: Note[] = [];
  /** Every rendering shown, oldest first, for `/panel`. */
  private galleries: Gallery[] = [];
  private lines: string[] = [];
  /** The last layout: each part's key, component and height in order, for scrolling to a message and routing clicks. */
  private layout: readonly { readonly key: string; readonly component: Component; readonly height: number }[] = [];
  private lastItem: string | null = null;
  private selected: string | null = null;
  private expanded = false;
  private readonly banner = new Container();
  private readonly omittedLine = line("");

  constructor(
    tui: TUI,
    cwd: () => string,
    source: () => TranscriptSource | null,
    openPanel: (gallery: Gallery, index: number) => void,
  ) {
    this.tui = tui;
    this.cwd = cwd;
    this.source = source;
    this.openPanel = openPanel;
    this.banner.addChild(line(muted("  Ask about this workspace, or what the running DimOS system can do.")));
    this.banner.addChild(new Spacer(1));
  }

  /** The newest rendering, which `/panel N` selects in. */
  get latestGallery(): Gallery | null {
    return this.galleries.at(-1) ?? null;
  }

  /** Reconciles to `source` and lays it out at `width`; returns each part's key and height in order. */
  show(source: TranscriptSource, width: number): readonly { readonly key: string; readonly height: number }[] {
    const { rows, omitted } = source;
    const parts: { readonly key: string; readonly component: Component }[] = [];
    if (rows.length === 0 && omitted === 0) parts.push({ key: "banner", component: this.banner });
    if (omitted > 0) {
      this.omittedLine.setText(
        muted(`  ${omitted} earlier items are not shown here; the whole conversation is kept on disk.`),
      );
      parts.push({ key: "omitted", component: this.omittedLine });
    }
    const items = new Set(rows.flatMap((row) => (row.kind === "item" ? [row.key] : [])));
    const anchored = new Map<string | null, Component[]>();
    for (const note of this.notes) {
      const after = note.after !== null && items.has(note.after) ? note.after : null;
      anchored.set(after, [...(anchored.get(after) ?? []), note.component]);
    }
    const placeNotes = (after: string | null) =>
      (anchored.get(after) ?? []).forEach((component, index) =>
        parts.push({ key: `note:${after ?? ""}:${index}`, component }),
      );
    placeNotes(null);
    const shown = new Set<string>();
    let lastItem: string | null = null;
    for (const row of rows) {
      shown.add(row.key);
      let view = this.views.get(row.key);
      if (view === undefined) {
        view = this.viewOf(row, rows);
        this.views.set(row.key, view);
      } else view.update(row);
      parts.push({ key: row.key, component: view.component });
      if (row.kind === "item") {
        lastItem = row.key;
        placeNotes(row.key);
      }
    }
    this.lastItem = lastItem;
    for (const [key, view] of this.views) {
      if (shown.has(key)) continue;
      view.dispose();
      this.views.delete(key);
    }
    this.lines = [];
    this.layout = parts.map(({ key, component }) => {
      const marked = key === this.selected;
      const lines = marked
        ? component.render(Math.max(1, width - SELECTED_GUTTER.length)).map((text) => accent(SELECTED_GUTTER) + text)
        : component.render(width);
      this.lines.push(...lines);
      return { key, component, height: lines.length };
    });
    return this.layout;
  }

  /** Marks the selected message with a gutter, or none. */
  select(key: string | null): void {
    this.selected = key;
    this.tui.requestRender();
  }

  /** The line the part with `key` starts on in the last layout, for a scroll view; null when it is not shown. */
  offsetOf(key: string): number | null {
    let offset = 0;
    for (const part of this.layout) {
      if (part.key === key) return offset;
      offset += part.height;
    }
    return null;
  }

  /** `/expand`: every card shows its full output. */
  setExpanded(expanded: boolean): void {
    this.expanded = expanded;
    for (const view of this.views.values()) view.setExpanded(expanded);
    this.tui.requestRender();
  }

  /** A rendering made in this terminal, such as `/inspect`'s; `/panel` selects in it like in any other. */
  showRendering(details: RenderDetails): void {
    const block = new Container();
    const gallery = this.gallery(block, details);
    this.notes.push({ after: this.lastItem, component: block, gallery });
    this.tui.requestRender();
  }

  /** A line the terminal adds itself, such as a command's answer or error. */
  note(text: string): void {
    this.notes.push({ after: this.lastItem, component: line(text), gallery: null });
    this.tui.requestRender();
  }

  /** Drops every component and note, for another conversation. */
  reset(): void {
    for (const view of this.views.values()) view.dispose();
    for (const note of this.notes) note.gallery?.dispose();
    this.views.clear();
    this.notes = [];
    this.galleries = [];
    this.lines = [];
    this.layout = [];
    this.lastItem = null;
    this.selected = null;
  }

  render(width: number): string[] {
    const source = this.source();
    if (source === null) return [];
    this.show(source, width);
    return this.lines;
  }

  /** Passes a mouse event to the part under it, such as a click on a gallery's tab row. */
  handleMouse(event: TuiMouseEvent): TuiMouseEventResult | undefined {
    let top = 0;
    for (const { key, component, height } of this.layout) {
      if (event.y < top + height) {
        const gutter = key === this.selected ? SELECTED_GUTTER.length : 0;
        return component.handleMouse?.({
          ...event,
          x: event.x - gutter,
          y: event.y - top,
          width: event.width - gutter,
          height,
        });
      }
      top += height;
    }
    return undefined;
  }

  invalidate(): void {
    for (const view of this.views.values()) view.component.invalidate();
    for (const note of this.notes) note.component.invalidate();
    this.banner.invalidate();
  }

  dispose(): void {
    this.reset();
  }

  private viewOf(row: TimelineRow, rows: readonly TimelineRow[]): RowView {
    switch (row.kind) {
      case "item":
        return this.itemView(row.item, row.outcome, rows);
      case "streaming": {
        let text = partialText(row.message);
        const markdown = new Markdown(text, 1, 1, getMarkdownTheme());
        return this.fixed(markdown, (next) => {
          if (next.kind !== "streaming" || partialText(next.message) === text) return;
          text = partialText(next.message);
          markdown.setText(text);
        });
      }
      case "tool":
        return this.toolView(row.call, row.slot.output ?? "", row.backgroundable);
      case "queued": {
        const text = line(queuedLine(row.row));
        return this.fixed(text, (next) => {
          if (next.kind === "queued") text.setText(queuedLine(next.row));
        });
      }
      case "sending":
        return this.fixed(line(muted(`   sending        ${clean(row.text)}`)), () => {});
      case "live_notice": {
        const text = line(muted(` ${clean(row.text)}`));
        return this.fixed(text, (next) => {
          if (next.kind === "live_notice") text.setText(muted(` ${clean(next.text)}`));
        });
      }
    }
  }

  private fixed(component: Component, update: (row: TimelineRow) => void): RowView {
    return { component, update, setExpanded: () => {}, dispose: () => {} };
  }

  private toolView(call: ToolCall, output: string, backgroundable: boolean): RowView {
    const card = cardFor(call, this.tui, this.cwd());
    card.setExpanded(this.expanded);
    if (output !== "") card.output(output);
    const hint = new Container();
    const block = new Container();
    block.addChild(card);
    block.addChild(hint);
    let shownOutput = output;
    let hinted = false;
    const showHint = (wanted: boolean) => {
      if (wanted === hinted) return;
      hinted = wanted;
      hint.clear();
      if (wanted) hint.addChild(line(muted("   Ctrl+B or /background moves this call to the background")));
    };
    showHint(backgroundable);
    return {
      component: block,
      update: (next) => {
        if (next.kind !== "tool") return;
        const nextOutput = next.slot.output ?? "";
        if (nextOutput !== shownOutput) {
          shownOutput = nextOutput;
          card.output(nextOutput);
        }
        showHint(next.backgroundable);
      },
      setExpanded: (expanded) => card.setExpanded(expanded),
      dispose: () => card.dispose(),
    };
  }

  private itemView(item: TranscriptItem, outcome: InputOutcome | null, rows: readonly TimelineRow[]): RowView {
    const block = new Container();
    const cards: ToolCard[] = [];
    const galleries: Gallery[] = [];
    switch (item.kind) {
      case "input":
        this.input(
          block,
          item.input,
          item.images.map((image) => image.mimeType),
          galleries,
        );
        break;
      case "user":
        block.addChild(new UserMessageComponent(userText(item.message)));
        break;
      case "assistant":
        block.addChild(new AssistantMessageComponent(cleanAssistant(item.message), true));
        break;
      case "tool_result": {
        const card = cardFor(callOf(rows, item.result), this.tui, this.cwd());
        card.setExpanded(this.expanded);
        card.end(withPlaceholders(item.result));
        block.addChild(card);
        cards.push(card);
        const render = renderOf(item.result);
        if (render !== null) galleries.push(this.gallery(block, render));
        break;
      }
      case "compaction":
        block.addChild(line(muted(` ⇣ Earlier context compacted: ${firstLine(item.summary)}`)));
        break;
      case "summary":
        block.addChild(
          line(
            muted(
              item.of === "up_to"
                ? ` ◇ Summary of #${item.from.conversation} before this point: ${firstLine(item.text)}`
                : ` ◇ Summary of a branch of #${item.from.conversation}: ${firstLine(item.text)}`,
            ),
          ),
        );
        break;
      case "reset":
        block.addChild(line(muted(" ─ context reset ─")));
        break;
    }
    const outcomeSlot = new Container();
    block.addChild(outcomeSlot);
    let shownOutcome = "";
    const showOutcome = (next: InputOutcome | null) => {
      if (outcomeKey(next) === shownOutcome) return;
      shownOutcome = outcomeKey(next);
      outcomeSlot.clear();
      if (next !== null && next.status === "unanswered")
        outcomeSlot.addChild(line(amber(` ✖ Unanswered: ${clean(next.reason)}`)));
    };
    showOutcome(outcome);
    return {
      component: block,
      update: (next) => {
        if (next.kind === "item") showOutcome(next.outcome);
      },
      setExpanded: (expanded) => {
        for (const card of cards) card.setExpanded(expanded);
      },
      dispose: () => {
        for (const card of cards) card.dispose();
        for (const gallery of galleries) gallery.dispose();
        this.galleries = this.galleries.filter((shown) => !galleries.includes(shown));
      },
    };
  }

  /** An input by its source: the user's text, "⚡ module: text", "⏰ job: text", "⇢ text", or only its runtime change. */
  private input(block: Container, input: DimosInput, images: readonly string[], galleries: Gallery[]): void {
    if (input.change !== null) block.addChild(line(muted(` ↻ runtime: ${clean(input.change.summary)}`)));
    const source = input.source;
    switch (source.kind) {
      case "user":
        block.addChild(new UserMessageComponent(cleanMultiline(input.text)));
        break;
      case "event":
        block.addChild(line(amber(" ⚡ ") + clean(`${source.event.module}: ${source.event.text}`)));
        break;
      case "job":
        block.addChild(line(accent(" ⏰ ") + clean(`${source.job}: ${input.text}`)));
        break;
      case "background":
        block.addChild(line(muted(" ⇢ ") + clean(`${source.tool} ${BACKGROUND_ENDS[source.end]} in the background`)));
        for (const text of input.text.split("\n").slice(0, BACKGROUND_LINES))
          block.addChild(line(muted(`   ${clean(text)}`)));
        break;
      case "runtime":
        return;
    }
    if (input.render !== null) galleries.push(this.gallery(block, input.render));
    else for (const mimeType of images) block.addChild(line(muted(`   [image ${mimeType}]`)));
  }

  private gallery(block: Container, details: RenderDetails): Gallery {
    const gallery = new Gallery(
      details,
      () => this.tui.requestRender(),
      (index) => this.openPanel(gallery, index),
    );
    this.galleries.push(gallery);
    block.addChild(gallery);
    return gallery;
  }
}
