import type { CardGroup } from "#dimos/agents/shared/view-models/streams/cards.ts";
import type { Channel, StreamRef } from "#dimos/agents/shared/protocol/streams.ts";
import type { Component } from "@earendil-works/pi-tui";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { FrameRecord, FrameSource } from "#dimos/agents/sdk/streams/base.ts";
import type { InboxState } from "@earendil-works/pi-durable";
import type { JobMark } from "#dimos/agents/shared/view-models/jobs/timeline.ts";
import type { JobRow } from "#dimos/agents/shared/protocol/jobs/state.ts";
import type { ModuleRow } from "#dimos/agents/shared/protocol/runtime/module.ts";
import type { OwnedStream, Transcript } from "#dimos/agents/shared/protocol/conversation/state.ts";
import type { RateTable } from "#dimos/agents/shared/view-models/streams/rates.ts";
import { FramePainter } from "#dimos/agents/frontends/tui/sidebar/painter.ts";
import {
  HStack,
  ScrollView,
  Text,
  allocateImageId,
  getCellDimensions,
  matchesKey,
  truncateToWidth,
} from "@earendil-works/pi-tui";
import { StreamCardView } from "#dimos/agents/frontends/tui/sidebar/card.ts";
import { accent, amber, levelColor, muted } from "#dimos/agents/frontends/tui/style.ts";
import { clean } from "#dimos/agents/shared/view-models/text.ts";
import { clock, duration } from "#dimos/agents/shared/view-models/time.ts";
import { ratesOf } from "#dimos/agents/shared/view-models/streams/rates.ts";
import { jobLanes } from "#dimos/agents/shared/view-models/jobs/timeline.ts";
import { noticesOf } from "#dimos/agents/shared/view-models/conversation/notices.ts";
import { sidebarGroups, systemGroups } from "#dimos/agents/shared/view-models/streams/cards.ts";

/** What the sidebar draws: the shown conversation's own rows, read when it renders. */
export type SidebarSource = {
  readonly streams: readonly OwnedStream[];
  readonly modules: readonly ModuleRow[];
  readonly jobs: readonly JobRow[];
  readonly transcript: Transcript;
  readonly queue: InboxState;
  /** Every stream and agent module of the running system, for the robot's own streams below the conversation's. */
  readonly system: { readonly streams: readonly StreamRef[]; readonly modules: readonly ModuleRow[] };
};

/** The column's width in cells, the "│ " gutter included. */
export const SIDEBAR_COLUMNS = 40;

const GUTTER = "│ ";

const INDENT = 2;

const PICTURE_COLUMNS = SIDEBAR_COLUMNS - GUTTER.length - INDENT;

const PICTURE_ROWS = 8;

const HOUR_MS = 60 * 60 * 1000;

const JOB_WINDOW_MS = 12 * HOUR_MS;

const RUNS_SHOWN = 3;

const RUN_MARKS = { submitted: "✓", ok: "✓", error: "✖", interrupted: "⚠" } as const;

type Row = { readonly key: string; readonly top: number; readonly bottom: number };

class SidebarViewport extends ScrollView {
  private readonly laidOut: () => void;

  constructor(content: Component, laidOut: () => void) {
    super(content, { follow: "none" });
    this.laidOut = laidOut;
  }

  override updateLayout(contentHeight: number, viewportHeight: number, requestRender: () => void): void {
    super.updateLayout(contentHeight, viewportHeight, requestRender);
    this.laidOut();
  }
}

function line(text: string): Text {
  return new Text(text, 0, 0);
}

function moduleHead(module: ModuleRow): string {
  const revision = module.revision === null ? "" : `r${module.revision} `;
  const owner = module.owner === null ? "" : ` · #${module.owner}`;
  return `${accent("⚙")} ${clean(module.name)}  ${revision}${module.state}${muted(owner)}`;
}

function markText(mark: JobMark): string {
  return mark.kind === "run"
    ? levelColor(mark.level, `${RUN_MARKS[mark.outcome]}${clock(mark.at)}`)
    : `○${clock(mark.at)}`;
}

/**
 * The collapsible column beside the chat (requirement 8), with what the conversation owns: each module with its state
 * and streams (requirement 2), its jobs (requirement 3) and its admitted notices (requirement 6). Cards are reconciled by channel, each with a fixed kitty image id, so a frame replaces
 * the picture in place. Rates come from the frame source's one-second samples; frames go through the painter one at a
 * time per channel, and a frame that arrives while one paints is dropped.
 */
export class Sidebar implements Component {
  readonly viewport: ScrollView;
  /** What the layout places: the gutter beside the viewport. */
  readonly column: HStack;
  private hasFocus = false;
  private readonly source: () => SidebarSource | null;
  private readonly frames: FrameSource;
  private readonly changed: () => void;
  private readonly painter: FramePainter;
  private readonly cards = new Map<Channel, StreamCardView>();
  private readonly painting = new Set<Channel>();
  private rows: readonly Row[] = [];
  private selected: string | null = null;
  private readonly collapsed = new Set<string>();
  private revealPending = false;
  private lastViewportHeight = 0;
  private visible = true;
  private rates: RateTable = new Map();
  private wanted = new Set<Channel>();
  private open = true;
  private closed = false;
  private readonly stopRecords: () => void;

  constructor(source: () => SidebarSource | null, frames: FrameSource, changed: () => void, screenRows: () => number) {
    this.source = source;
    this.frames = frames;
    this.changed = changed;
    this.viewport = new SidebarViewport(this, () => this.afterLayout());
    // A leaf only learns its width, so the gutter draws a row for every screen row and the layout clips it to the
    // column's height.
    const gutter: Component = {
      render: () => Array.from({ length: screenRows() }, () => muted(GUTTER)),
      invalidate: () => {},
    };
    this.column = new HStack([
      { component: gutter, basis: GUTTER.length },
      { component: this.viewport, basis: 0, grow: 1 },
    ]);
    const cell = getCellDimensions();
    this.painter = new FramePainter({ width: PICTURE_COLUMNS * cell.widthPx, height: PICTURE_ROWS * cell.heightPx });
    this.stopRecords = frames.onRecord((record) => this.receive(record));
  }

  get isOpen(): boolean {
    return this.open;
  }

  get isVisible(): boolean {
    return this.open && this.visible;
  }

  get focused(): boolean {
    return this.hasFocus;
  }

  set focused(focused: boolean) {
    if (this.hasFocus === focused) return;
    this.hasFocus = focused;
    if (focused) this.revealPending = true;
    this.changed();
  }

  /** Ctrl+T. A closed sidebar wants no frames. */
  setOpen(open: boolean): void {
    if (this.open === open) return;
    this.open = open;
    if (!open) this.want(new Set());
    this.changed();
  }

  setVisible(visible: boolean): void {
    if (this.visible === visible) return;
    this.visible = visible;
    if (!visible) this.want(new Set());
    else this.changed();
  }

  handleInput(data: string): void {
    if (!this.focused || this.rows.length === 0) return;
    const index = this.rows.findIndex((row) => row.key === this.selected);
    if (matchesKey(data, "up") || matchesKey(data, "down")) {
      const next = Math.max(0, Math.min(this.rows.length - 1, index + (matchesKey(data, "up") ? -1 : 1)));
      this.selected = this.rows[next]?.key ?? null;
      this.revealPending = true;
      this.changed();
    } else if (this.selected !== null && matchesKey(data, "left")) {
      this.collapsed.add(this.selected);
      this.revealPending = true;
      this.changed();
    } else if (this.selected !== null && matchesKey(data, "right")) {
      this.collapsed.delete(this.selected);
      this.revealPending = true;
      this.changed();
    }
  }

  render(width: number): string[] {
    const source = this.source();
    const now = Date.now() as EpochMs;
    const content: string[] = [];
    const add = (component: Component): void => {
      content.push(...component.render(width));
    };
    const rows: Row[] = [];
    const shown = new Set<Channel>();
    const wanted = new Set<Channel>();
    const occurrences = new Map<string, number>();
    if (source === null) {
      add(line(muted("no conversation")));
    } else {
      const own = sidebarGroups(source.streams, source.modules, this.rates, now);
      this.showGroups(own, "owned", now, width, content, rows, shown, wanted, occurrences);
      this.showJobs(source.jobs, now, add);
      this.showNotices(source, add);
      const system = systemGroups(source.system.streams, source.system.modules, shown, this.rates, now);
      if (system.groups.length > 0) {
        add(line(`${accent("◉")} system`));
        this.showGroups(system.groups, "system", now, width, content, rows, shown, wanted, occurrences);
      }
      if (system.others > 0) add(line(muted(`  + ${system.others} more streams in /menu`)));
    }
    const valid = new Set(rows.map((row) => row.key));
    for (const key of this.collapsed) if (!valid.has(key)) this.collapsed.delete(key);
    const nextSelected = this.selected !== null && valid.has(this.selected) ? this.selected : (rows[0]?.key ?? null);
    if (nextSelected !== this.selected) {
      this.selected = nextSelected;
      this.revealPending = true;
      if (this.focused) this.changed();
    }
    const prior = this.rows.find((row) => row.key === this.selected);
    const current = rows.find((row) => row.key === this.selected);
    if (this.focused && (prior?.top !== current?.top || prior?.bottom !== current?.bottom)) this.revealPending = true;
    this.rows = rows;
    this.prune(shown);
    this.want(this.open && this.visible && source !== null ? wanted : new Set());
    add(line(muted(this.focused ? "↑↓ move · ←→ fold · Esc chat" : "Shift+Tab streams · Ctrl+T hides")));
    return content.map((text) => truncateToWidth(text, width));
  }

  invalidate(): void {
    for (const card of this.cards.values()) card.invalidate();
  }

  dispose(): void {
    this.closed = true;
    this.stopRecords();
    this.want(new Set());
    for (const card of this.cards.values()) card.dispose();
    this.cards.clear();
  }

  private showGroups(
    groups: readonly CardGroup[],
    section: "owned" | "system",
    now: EpochMs,
    width: number,
    content: string[],
    rows: Row[],
    shown: Set<Channel>,
    wanted: Set<Channel>,
    occurrences: Map<string, number>,
  ): void {
    const add = (component: Component): void => {
      content.push(...component.render(width));
    };
    for (const group of groups) {
      if (group.module !== null) {
        add(line(moduleHead(group.module)));
        if (group.module.lastError !== null)
          add(line(amber(`  ✖ ${clean(group.module.lastError.split("\n")[0] ?? "")}`)));
      } else {
        add(line(` ${clean(group.title)}`));
      }
      for (const card of group.cards) {
        const base = JSON.stringify([
          section,
          group.module?.instance ?? group.title,
          card.channel,
          card.direction,
          card.title,
        ]);
        const ordinal = occurrences.get(base) ?? 0;
        occurrences.set(base, ordinal + 1);
        const key = `${base}:${ordinal}`;
        shown.add(card.channel);
        let view = this.cards.get(card.channel);
        if (view === undefined) {
          view = new StreamCardView(allocateImageId(), PICTURE_COLUMNS, PICTURE_ROWS, this.changed);
          this.cards.set(card.channel, view);
        }
        view.update(card, now);
        const top = content.length;
        const expanded = !this.collapsed.has(key);
        content.push(...view.render(width, this.focused && this.selected === key, expanded));
        rows.push({ key, top, bottom: content.length });
        if (expanded) wanted.add(card.channel);
      }
    }
  }

  private afterLayout(): void {
    if (this.viewport.viewportHeight !== this.lastViewportHeight) {
      this.lastViewportHeight = this.viewport.viewportHeight;
      if (this.focused) this.revealPending = true;
    }
    if (!this.revealPending || !this.focused) return;
    this.revealPending = false;
    const row = this.rows.find((candidate) => candidate.key === this.selected);
    if (row === undefined || this.viewport.viewportHeight === 0) return;
    const top = this.viewport.scrollTop;
    const height = this.viewport.viewportHeight;
    if (row.top < top || row.bottom - row.top > height) this.viewport.scrollTo(row.top);
    else if (row.bottom > top + height) this.viewport.scrollTo(row.bottom - height);
  }

  private prune(shown: ReadonlySet<Channel>): void {
    for (const channel of [...this.cards.keys()]) {
      if (shown.has(channel)) continue;
      this.cards.get(channel)?.dispose();
      this.cards.delete(channel);
    }
  }

  private showJobs(jobs: readonly JobRow[], now: EpochMs, add: (component: Component) => void): void {
    const lanes = jobLanes(jobs, (now - JOB_WINDOW_MS) as EpochMs, (now + JOB_WINDOW_MS) as EpochMs);
    for (const lane of lanes) {
      const next = lane.row.nextDue === null ? "" : `  next ${clock(lane.row.nextDue)}`;
      add(line(`${accent("⏰")} ${clean(lane.row.name)}  ${muted(clean(lane.row.cron))}${next}`));
      const runs = lane.marks.filter((mark) => mark.kind === "run").slice(-RUNS_SHOWN);
      if (runs.length > 0) add(line(`  ${runs.map(markText).join(" ")}`));
    }
  }

  private showNotices(source: SidebarSource, add: (component: Component) => void): void {
    const notices = noticesOf(source.transcript, source.queue);
    const parts: string[] = [];
    if (notices.queued.length > 0) parts.push(`${notices.queued.length} queued`);
    if (parts.length === 0) return;
    add(line(amber("⚡ ") + `notices: ${parts.join(" · ")}`));
  }

  private want(channels: ReadonlySet<Channel>): void {
    if (channels.size === this.wanted.size && [...channels].every((channel) => this.wanted.has(channel))) return;
    this.wanted = new Set(channels);
    this.rates = new Map([...this.rates].filter(([channel]) => channels.has(channel)));
    this.frames.want(this.wanted);
  }

  private receive(record: FrameRecord): void {
    if (this.closed) return;
    switch (record.kind) {
      case "rates":
        this.rates = ratesOf(record.sample);
        this.changed();
        return;
      case "value":
        this.cards.get(record.channel)?.setValue(record.text);
        this.changed();
        return;
      case "error":
        this.cards.get(record.channel)?.setError(record.text);
        this.changed();
        return;
      case "image":
      case "cloud":
      case "grid": {
        const view = this.cards.get(record.channel);
        if (this.painting.has(record.channel) || view === undefined) return;
        this.painting.add(record.channel);
        this.painter.paint(record).then(
          (png) => {
            this.painting.delete(record.channel);
            if (this.closed || this.cards.get(record.channel) !== view) return;
            view.setFrame(png);
            this.changed();
          },
          () => this.painting.delete(record.channel),
        );
        return;
      }
    }
  }
}
