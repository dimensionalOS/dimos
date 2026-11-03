import type { CallId } from "#dimos/agents/shared/protocol/conversation/calls.ts";
import type { Component } from "@earendil-works/pi-tui";
import type { MovedRow } from "#dimos/agents/shared/view-models/conversation/tools.ts";
import { accent, muted } from "#dimos/agents/frontends/tui/style.ts";
import { clean } from "#dimos/agents/shared/view-models/text.ts";
import { duration } from "#dimos/agents/shared/view-models/time.ts";
import { matchesKey, truncateToWidth, visibleWidth } from "@earendil-works/pi-tui";

/** Rows shown at once; the selection scrolls the rest into view. */
const VISIBLE_ROWS = 5;

export type BackgroundActions = {
  /** Shows the call's output in place of the chat. */
  readonly open: (call: CallId) => void;
  readonly stop: (call: CallId) => void;
  /** Gives focus back to the editor. */
  readonly leave: () => void;
};

/**
 * The calls running in the background, under the editor, while there are any: one line for the newest until Down on an
 * empty editor focuses it, then the list. Up/Down select, Enter opens the output, x stops the call, Esc or Up past the
 * first row goes back to the editor.
 */
export class BackgroundPanel implements Component {
  private readonly rows: () => readonly MovedRow[];
  private readonly actions: BackgroundActions;
  private hasFocus = false;
  private selected: CallId | null = null;

  constructor(rows: () => readonly MovedRow[], actions: BackgroundActions) {
    this.rows = rows;
    this.actions = actions;
  }

  get focused(): boolean {
    return this.hasFocus;
  }

  set focused(focused: boolean) {
    this.hasFocus = focused;
  }

  get isEmpty(): boolean {
    return this.rows().length === 0;
  }

  handleInput(data: string): void {
    const rows = this.rows();
    const index = this.indexIn(rows);
    const row = rows[index];
    if (row === undefined || matchesKey(data, "escape")) this.actions.leave();
    else if (matchesKey(data, "up")) {
      if (index === 0) this.actions.leave();
      else this.selected = rows[index - 1]?.call ?? null;
    } else if (matchesKey(data, "down")) this.selected = rows[Math.min(rows.length - 1, index + 1)]?.call ?? null;
    else if (matchesKey(data, "enter")) this.actions.open(row.call);
    else if (data === "x") this.actions.stop(row.call);
  }

  render(width: number): string[] {
    const rows = this.rows();
    if (rows.length === 0) {
      if (this.hasFocus) this.actions.leave();
      return [];
    }
    if (!this.hasFocus) {
      const newest = rows[rows.length - 1];
      const more = rows.length > 1 ? ` · ${rows.length - 1} more` : "";
      const hint = muted(`${more} · ↓ to manage`);
      return newest === undefined
        ? []
        : [truncateToWidth(`  ${describe(newest)}`, Math.max(0, width - visibleWidth(hint))) + hint];
    }
    const index = this.indexIn(rows);
    this.selected = rows[index]?.call ?? null;
    const start = Math.max(0, Math.min(rows.length - VISIBLE_ROWS, index - Math.floor(VISIBLE_ROWS / 2)));
    const lines = rows
      .slice(start, start + VISIBLE_ROWS)
      .map((row, offset) => `${start + offset === index ? accent("› ") : "  "}${describe(row)}`);
    const count = rows.length > VISIBLE_ROWS ? ` (${index + 1}/${rows.length})` : "";
    const hint = muted(`  ↑↓ move · Enter output · x stop · Esc chat${count}`);
    return [...lines, hint].map((line) => truncateToWidth(line, width));
  }

  invalidate(): void {}

  /** The selected row's index, the first row when the selection ended or was never made. */
  private indexIn(rows: readonly MovedRow[]): number {
    const at = rows.findIndex((row) => row.call === this.selected);
    return at === -1 ? 0 : at;
  }
}

function describe(row: MovedRow): string {
  const output = row.lastLine === "" ? "" : muted(` · ${clean(row.lastLine)}`);
  return `${accent("⇢")} ${clean(row.label)}${muted(` · ${duration(row.forMs)}`)}${output}`;
}
