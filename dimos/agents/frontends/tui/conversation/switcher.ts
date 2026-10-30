import type { Component } from "@earendil-works/pi-tui";
import type { ConversationId } from "@earendil-works/pi-durable";
import type { Role } from "../../../sdk/state.ts";
import type { SwitcherRow } from "../../../shared/view-models/gateway/switcher.ts";
import { SelectList, matchesKey, truncateToWidth } from "@earendil-works/pi-tui";
import { accent, muted } from "../style.ts";
import { getSelectListTheme } from "@earendil-works/pi-coding-agent";
import { switcherLine } from "../../../shared/view-models/gateway/switcher.ts";

const ROWS_SHOWN = 12;

const FILTER_KEYS = ["backspace", "ctrl+h"] as const;

/** What the switcher does on a pick; the terminal closes the overlay first. */
export type SwitcherActions = {
  attach(conversation: ConversationId, role: Role): void;
  create(): void;
  close(): void;
};

/**
 * The conversation switcher (requirement 9), shown as an overlay: every conversation with its state, what runs, its
 * unread notices and who is attached, forks indented under their source. Typing filters by the row's line,
 * Enter attaches as owner, Tab as viewer, Ctrl+N starts a new conversation, Esc closes. Rows are read when it renders,
 * so a run that ends while the list is open updates its line.
 */
export class SwitcherOverlay implements Component {
  private readonly rows: () => readonly SwitcherRow[];
  private readonly actions: SwitcherActions;
  private list = new SelectList([], ROWS_SHOWN, getSelectListTheme());
  /** The lines of the rows `list` shows, joined; null before the first build. */
  private lines: string | null = null;
  private filter = "";

  constructor(rows: () => readonly SwitcherRow[], actions: SwitcherActions) {
    this.rows = rows;
    this.actions = actions;
    this.sync(rows());
  }

  handleInput(data: string): void {
    if (matchesKey(data, "escape")) {
      this.actions.close();
      return;
    }
    if (matchesKey(data, "tab")) {
      const picked = this.list.getSelectedItem();
      if (picked !== null) this.actions.attach(Number(picked.value) as ConversationId, "viewer");
      return;
    }
    if (matchesKey(data, "ctrl+n")) {
      this.actions.create();
      return;
    }
    if (FILTER_KEYS.some((key) => matchesKey(data, key))) {
      this.filter = this.filter.slice(0, -1);
      this.sync(this.rows());
      return;
    }
    if (data.length === 1 && data >= " " && data !== "\x7f") {
      this.filter += data;
      this.sync(this.rows());
      return;
    }
    this.list.handleInput(data);
  }

  render(width: number): string[] {
    const rows = this.rows();
    this.sync(rows);
    const filter = this.filter === "" ? "" : `  filter: ${this.filter}`;
    const body =
      rows.length === 0
        ? [muted(" No conversations")]
        : this.lines === ""
          ? [muted(" No matching conversations")]
          : this.list.render(width);
    return [
      truncateToWidth(` ${accent("Conversations")}${muted(filter)}`, width),
      ...body,
      truncateToWidth(muted(" Enter attach · Tab view · Ctrl+N new · Esc close · type to filter"), width),
    ];
  }

  invalidate(): void {
    this.list.invalidate();
  }

  /** Rebuilds the list from the rows whose line contains the filter when those lines change, keeping the selected row. */
  private sync(rows: readonly SwitcherRow[]): void {
    const filter = this.filter.toLowerCase();
    const shown = rows
      .map((row) => ({ value: String(row.id), label: switcherLine(row) }))
      .filter((item) => item.label.toLowerCase().includes(filter));
    const lines = shown.map((item) => item.label).join("\n");
    if (lines === this.lines) return;
    const selected = this.list.getSelectedItem()?.value;
    this.lines = lines;
    this.list = new SelectList(shown, ROWS_SHOWN, getSelectListTheme());
    this.list.setSelectedIndex(
      Math.max(
        0,
        shown.findIndex((item) => item.value === selected),
      ),
    );
    this.list.onSelect = (item) => this.actions.attach(Number(item.value) as ConversationId, "owner");
    this.list.onCancel = () => this.actions.close();
  }
}
