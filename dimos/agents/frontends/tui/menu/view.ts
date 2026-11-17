import { refusalOfFailure } from "#dimos/agents/sdk/failures.ts";
import type { Component } from "@earendil-works/pi-tui";
import type { ConversationId } from "@earendil-works/pi-durable";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { MenuDetail, MenuRow, MenuTab, TabContext } from "#dimos/agents/frontends/tui/menu/base.ts";
import type { Ui } from "#dimos/agents/frontends/tui/session.ts";
import { accent, levelColor, muted } from "#dimos/agents/frontends/tui/style.ts";
import { actionLine } from "#dimos/agents/shared/view-models/actions.ts";
import { clean } from "#dimos/agents/shared/view-models/text.ts";
import { conversationsTab } from "#dimos/agents/frontends/tui/menu/conversations.ts";
import { connectionSegment } from "#dimos/agents/shared/view-models/connection.ts";
import { linkSegments, runSegment } from "#dimos/agents/shared/view-models/gateway/link.ts";
import { jobsHeading, jobsTab } from "#dimos/agents/frontends/tui/menu/jobs.ts";
import { listen } from "#dimos/agents/sdk/state.ts";
import { decodeKittyPrintable, matchesKey, truncateToWidth, wrapTextWithAnsi } from "@earendil-works/pi-tui";
import { modulesTab } from "#dimos/agents/frontends/tui/menu/modules.ts";

const TABS: readonly MenuTab[] = [conversationsTab, modulesTab, jobsTab];
const TITLES = TABS.map((tab) => tab.title);

type Action = { label: string; run(): void | Promise<void> };
type Item = { kind: "module" | "job"; name: string };
type ItemScreen = { kind: "item"; item: Item; selected: number };
type Screen =
  | { kind: "list" }
  | ItemScreen
  | { kind: "detail"; detail: MenuDetail; parent: ItemScreen; scroll: number };

export type MenuMemory = { tab: number; selections: number[]; filters: string[] };
export function newMenuMemory(): MenuMemory {
  return { tab: 0, selections: [0, 0, 0], filters: ["", "", ""] };
}

type MenuViewOptions = {
  readonly ui: Ui;
  readonly current: ConversationId | null;
  readonly attach: (conversation: ConversationId) => void;
  readonly close: () => void;
  readonly requestRender: () => void;
  readonly height: () => number;
  readonly memory: MenuMemory;
};

export class MenuView implements Component {
  private readonly stop: () => void;
  private readonly options: MenuViewOptions;
  private screen: Screen = { kind: "list" };

  constructor(options: MenuViewOptions) {
    this.options = options;
    const { ui } = options;
    this.stop = listen(
      [
        ui.gateway.link,
        ui.gateway.load,
        ui.gateway.runs,
        ui.gateway.conversations,
        ui.gateway.modules,
        ui.gateway.jobs,
        ui.actions,
        ui.connection,
      ],
      () => options.requestRender(),
    );
  }

  handleInput(data: string): void {
    const screen = this.screen;
    if (matchesKey(data, "escape")) {
      if (screen.kind === "detail") this.screen = screen.parent;
      else if (screen.kind === "item") this.screen = { kind: "list" };
      else this.options.close();
    } else if (screen.kind === "detail") {
      if (matchesKey(data, "up")) screen.scroll = Math.max(0, screen.scroll - 1);
      if (matchesKey(data, "down")) screen.scroll++;
    } else if (screen.kind === "item") {
      const actions = this.actions(screen);
      screen.selected = Math.min(screen.selected, Math.max(0, actions.length - 1));
      if (matchesKey(data, "up") || matchesKey(data, "down"))
        screen.selected = Math.max(
          0,
          Math.min(actions.length - 1, screen.selected + (matchesKey(data, "down") ? 1 : -1)),
        );
      else if (matchesKey(data, "enter")) this.run(actions[screen.selected]);
    } else if (matchesKey(data, "left") || matchesKey(data, "right")) {
      const direction = matchesKey(data, "right") ? 1 : -1;
      this.options.memory.tab = (this.tab + direction + TABS.length) % TABS.length;
    } else if (matchesKey(data, "up") || matchesKey(data, "down")) {
      this.selected = Math.max(
        0,
        Math.min(this.items().length - 1, this.selected + (matchesKey(data, "down") ? 1 : -1)),
      );
    } else if (matchesKey(data, "enter")) this.open();
    else {
      const character = decodeKittyPrintable(data) ?? data;
      if (matchesKey(data, "backspace") || matchesKey(data, "ctrl+h"))
        this.filter = [...this.filter].slice(0, -1).join("");
      else if ([...character].length === 1 && character >= " " && character !== "\x7f") this.filter += character;
      else return;
      this.selected = 0;
    }
    this.options.requestRender();
  }

  render(width: number): string[] {
    const tabs = TITLES.map((title, index) => (index === this.tab ? accent(`[${title}]`) : muted(` ${title} `))).join(
      " ",
    );
    const { ui } = this.options;
    const runs = ui.gateway.runs.value;
    const run = runSegment(runs);
    const status = [
      connectionSegment(ui.connection.value),
      ...linkSegments(ui.gateway.link.value, ui.gateway.load.value, runs, Date.now() as EpochMs),
      ...(run === null ? [] : [run]),
    ];
    const statusLine = status.map((segment) => levelColor(segment.level, clean(segment.text))).join(muted(" · "));
    const lines = [tabs];
    const withFooter = (footer: readonly string[], focusedLine?: number): string[] => {
      const height = Math.max(0, this.options.height());
      const footerHeight = focusedLine === undefined ? height : Math.max(0, height - 1);
      const bottom = footer.slice(Math.max(0, footer.length - footerHeight));
      const bodyHeight = height - bottom.length;
      const start = Math.max(0, (focusedLine ?? 0) - bodyHeight + 1);
      const body = lines.slice(start, start + bodyHeight);
      while (body.length < bodyHeight) body.push("");
      return [...body, ...bottom].map((line) => truncateToWidth(line, width));
    };
    const newest = ui.actions.value.at(-1);
    const observation = newest === undefined ? "" : clean(actionLine(newest));
    const screen = this.screen;
    if (screen.kind === "detail") {
      const detailLines = screen.detail.lines().flatMap((line) => wrapTextWithAnsi(clean(line), width));
      const available = Math.max(1, this.options.height() - 6);
      screen.scroll = Math.min(screen.scroll, Math.max(0, detailLines.length - available));
      lines.push(
        "",
        accent(clean(`Jobs › ${screen.detail.title}`)),
        ...detailLines.slice(screen.scroll, screen.scroll + available),
      );
      return withFooter([observation, muted("↑/↓ scroll · Esc back"), statusLine]);
    }
    if (screen.kind === "item") {
      const row = this.row(screen.item);
      const definition = row === undefined ? ["This item is gone."] : [row.text];
      const actions = this.actions(screen);
      screen.selected = Math.min(screen.selected, Math.max(0, actions.length - 1));
      const title = screen.item.name;
      const available = Math.max(0, this.options.height() - actions.length - 8);
      lines.push(
        "",
        accent(clean(`${TITLES[this.tab]} › ${title}`)),
        ...definition.map((line) => truncateToWidth(clean(line), width)).slice(0, available),
        "",
        ...actions.map((action, index) =>
          index === screen.selected ? accent(`› ${clean(action.label)}`) : `  ${clean(action.label)}`,
        ),
      );
      return withFooter(
        [observation, muted("↑/↓ select action · Enter run · Esc back"), statusLine],
        actions.length ? lines.length - actions.length + screen.selected : undefined,
      );
    }
    const items = this.items();
    this.selected = Math.min(this.selected, Math.max(0, items.length - 1));
    const footer = [
      observation,
      muted("←/→ sections · ↑/↓ select · Enter open · type to filter · Esc back"),
      statusLine,
    ];
    const available = Math.max(
      1,
      this.options.height() - lines.length - footer.length - (TABS[this.tab] === jobsTab ? 2 : 1),
    );
    const start = Math.max(0, Math.min(this.selected - Math.floor(available / 2), items.length - available));
    const list =
      items.length === 0
        ? [muted("  No matches")]
        : items
            .slice(start, start + available)
            .map(
              (item, offset) =>
                (start + offset === this.selected ? accent("› ") : "  ") + levelColor(item.level, clean(item.text)),
            );
    lines.push(muted(`Search: ${this.filter || "(type to filter)"}`));
    if (TABS[this.tab] === jobsTab) lines.push(muted(jobsHeading(Date.now() as EpochMs)));
    lines.push(...list);
    return withFooter(footer);
  }

  invalidate(): void {}
  dispose(): void {
    this.stop();
  }
  private get tab(): number {
    return this.options.memory.tab;
  }
  private get selected(): number {
    return this.options.memory.selections[this.tab] ?? 0;
  }
  private set selected(value: number) {
    this.options.memory.selections[this.tab] = value;
  }
  private get filter(): string {
    return this.options.memory.filters[this.tab] ?? "";
  }
  private set filter(value: string) {
    this.options.memory.filters[this.tab] = value;
  }

  private context(): TabContext {
    return {
      ui: this.options.ui,
      now: Date.now() as EpochMs,
      current: this.options.current,
      attach: this.options.attach,
    };
  }
  private items(): readonly MenuRow[] {
    const tab = TABS[this.tab];
    return (tab?.rows(this.context()) ?? []).filter((row) =>
      `${row.text} ${row.searchText ?? ""}`.toLowerCase().includes(this.filter.toLowerCase()),
    );
  }
  private open(): void {
    this.selected = Math.min(this.selected, Math.max(0, this.items().length - 1));
    const row = this.items()[this.selected];
    if (row === undefined) return;
    if (TABS[this.tab] === conversationsTab) {
      this.run({
        label: "Resume conversation",
        run: () => {
          row.open?.();
        },
      });
      return;
    }
    if (row.searchText === undefined) return;
    const item: Item = { kind: TABS[this.tab] === modulesTab ? "module" : "job", name: row.searchText };
    this.screen = { kind: "item", item, selected: 0 };
  }
  private row(item: Item): MenuRow | undefined {
    const tab = item.kind === "module" ? modulesTab : jobsTab;
    return tab.rows(this.context()).find((row) => row.searchText === item.name);
  }
  private actions(screen: ItemScreen): Action[] {
    const row = this.row(screen.item);
    if (row === undefined) return [];
    const actions: Action[] = [];
    if (screen.item.kind === "job" && row.open !== null)
      actions.push({
        label: "View history",
        run: () => {
          const detail = row.open?.();
          if (detail) this.screen = { kind: "detail", detail, parent: screen, scroll: 0 };
        },
      });
    return [
      ...actions,
      ...row.actions.map((action) => ({ ...action, label: action.label[0]?.toUpperCase() + action.label.slice(1) })),
    ];
  }
  private run(action: Action | undefined): void {
    if (action === undefined) return;
    Promise.resolve()
      .then(() => action.run())
      .catch((error) => this.options.ui.actions.refuse(action.label, refusalOfFailure(error, Date.now() as EpochMs)))
      .finally(() => this.options.requestRender());
  }
}
