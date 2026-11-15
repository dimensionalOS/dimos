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
import {
  environmentTab,
  inventoryModulesTab,
  inventoryStreamsTab,
  rpcsTab,
} from "#dimos/agents/frontends/tui/menu/inventory.ts";
import { jobsTab } from "#dimos/agents/frontends/tui/menu/jobs.ts";
import { listen } from "#dimos/agents/sdk/state.ts";
import { matchesKey, truncateToWidth } from "@earendil-works/pi-tui";
import { modulesTab } from "#dimos/agents/frontends/tui/menu/modules.ts";
import { overviewTab } from "#dimos/agents/frontends/tui/menu/overview.ts";
import { streamsTab } from "#dimos/agents/frontends/tui/menu/streams.ts";

/** The family's one list, in key order 1 to 9. */
const TABS: readonly MenuTab[] = [
  overviewTab,
  conversationsTab,
  modulesTab,
  jobsTab,
  streamsTab,
  environmentTab,
  inventoryModulesTab,
  rpcsTab,
  inventoryStreamsTab,
];

const TAB_KEYS = ["1", "2", "3", "4", "5", "6", "7", "8", "9"];

type MenuViewOptions = {
  readonly ui: Ui;
  readonly inventory: TabContext["inventory"];
  /** The conversation the chat shows. */
  readonly current: ConversationId | null;
  /** Shows a conversation in the chat; /menu closes first. */
  readonly attach: (conversation: ConversationId) => void;
  readonly close: () => void;
  readonly requestRender: () => void;
  /** The screen rows the menu fills below the status line. */
  readonly height: () => number;
};

/**
 * Full-screen /menu: tabs 1 to 9, a selected row with its keys, Enter for its detail, Esc back then out. Draws from
 * the gateway members when it renders and reads the inventory held by the terminal.
 */
export class MenuView implements Component {
  private readonly options: MenuViewOptions;
  private tab = 0;
  private selected = 0;
  private detail: MenuDetail | null = null;
  private readonly stop: () => void;

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
        ui.gateway.streams,
        ui.gateway.jobs,
        ui.actions,
        ui.connection,
      ],
      () => options.requestRender(),
    );
  }

  handleInput(data: string): void {
    if (matchesKey(data, "escape")) {
      if (this.detail !== null) this.closeDetail();
      else this.options.close();
      return;
    }
    if (this.detail !== null) return;
    const tab = TAB_KEYS.indexOf(data);
    if (tab >= 0) {
      this.tab = tab;
      this.selected = 0;
    } else if (matchesKey(data, "up")) this.selected = Math.max(0, this.selected - 1);
    else if (matchesKey(data, "down")) this.selected += 1;
    else if (matchesKey(data, "enter")) {
      const open = this.rows()[this.selected]?.open;
      if (open !== null && open !== undefined) this.detail = open();
    } else {
      const key = this.rows()[this.selected]?.keys.find((candidate) => candidate.key === data);
      if (key !== undefined)
        void key
          .run()
          .catch((error) => this.options.ui.actions.refuse(key.label, refusalOfFailure(error, Date.now() as EpochMs)));
    }
    this.options.requestRender();
  }

  render(width: number): string[] {
    const tabs = TABS.map((tab, index) =>
      index === this.tab ? accent(`[${TAB_KEYS[index]} ${tab.title}]`) : muted(` ${TAB_KEYS[index]} ${tab.title} `),
    ).join("");
    const lines = [tabs, ""];
    const detail = this.detail;
    if (detail !== null) {
      lines.push(accent(clean(detail.title)), ...detail.lines().map((line) => clean(line)));
      lines.push("", muted("Esc back"));
    } else {
      const rows = this.rows();
      this.selected = Math.min(this.selected, Math.max(0, rows.length - 1));
      const newest = this.options.ui.actions.value.at(-1);
      const footer = [""];
      if (
        newest !== undefined &&
        !(TABS[this.tab] === overviewTab && (newest.status === "refused" || newest.status === "unknown"))
      )
        footer.push(clean(actionLine(newest)));
      footer.push(muted(this.hints(rows[this.selected])));
      if (rows.length > Math.max(1, this.options.height() - lines.length - footer.length))
        footer.unshift(muted(`  (${this.selected + 1}/${rows.length})`));
      const visible = Math.max(1, Math.min(rows.length, this.options.height() - lines.length - footer.length));
      const start = Math.max(0, Math.min(this.selected - Math.floor(visible / 2), rows.length - visible));
      if (rows.length === 0) lines.push(muted("None"));
      rows
        .slice(start, start + visible)
        .forEach((row, offset) =>
          lines.push((start + offset === this.selected ? accent("› ") : "  ") + levelColor(row.level, row.text)),
        );
      lines.push(...footer);
    }
    return lines.map((line) => truncateToWidth(line, width));
  }

  invalidate(): void {}

  dispose(): void {
    this.stop();
    this.closeDetail();
  }

  private hints(row: MenuRow | undefined): string {
    const keys = (row?.keys ?? []).map((key) => `${key.key} ${key.label}`);
    if (row?.open !== null && row?.open !== undefined) keys.push("Enter open");
    return ["1–9 tab", "↑/↓ select", ...keys, "Esc chat"].join(" · ");
  }

  private rows(): readonly MenuRow[] {
    const tab = TABS[this.tab];
    if (tab === undefined) return [];
    const context: TabContext = {
      ui: this.options.ui,
      now: Date.now() as EpochMs,
      inventory: this.options.inventory,
      current: this.options.current,
      attach: (conversation) => this.options.attach(conversation),
    };
    return tab.rows(context);
  }

  private closeDetail(): void {
    void this.detail?.close();
    this.detail = null;
  }
}
