import type { Component } from "@earendil-works/pi-tui";
import type { Selectable } from "../../../shared/view-models/conversation/history.ts";
import { accent, muted } from "../style.ts";
import { matchesKey, truncateToWidth } from "@earendil-works/pi-tui";

export type SelectionChoice =
  | { readonly kind: "edit"; readonly target: Selectable }
  | { readonly kind: "summarize_up_to"; readonly target: Selectable }
  | { readonly kind: "summarize_from"; readonly target: Selectable }
  | { readonly kind: "cancel" };

type SelectionActions = {
  /** The arrows moved: the terminal marks and reveals the message. */
  moved(target: Selectable): void;
  choose(choice: SelectionChoice): void;
};

/**
 * Esc-then-arrows message selection (requirement 13). Takes the editor's place while it is focused: the arrows move
 * over `selectables`, newest first; on the selected message, e edits and resends, u summarizes up to it and f from it,
 * each only where the message allows it; Esc returns to the editor.
 */
export class MessageSelection implements Component {
  private readonly targets: readonly Selectable[];
  private readonly actions: SelectionActions;
  private index: number;

  constructor(targets: readonly Selectable[], actions: SelectionActions) {
    this.targets = targets;
    this.actions = actions;
    this.index = targets.length - 1;
  }

  get target(): Selectable | null {
    return this.targets[this.index] ?? null;
  }

  handleInput(data: string): void {
    if (matchesKey(data, "escape")) {
      this.actions.choose({ kind: "cancel" });
      return;
    }
    if (matchesKey(data, "up") || matchesKey(data, "down")) {
      const next = this.index + (matchesKey(data, "up") ? -1 : 1);
      if (next < 0 || next >= this.targets.length) return;
      this.index = next;
      const target = this.target;
      if (target !== null) this.actions.moved(target);
      return;
    }
    const target = this.target;
    if (target === null) return;
    if (data === "e" && target.edit !== null) this.actions.choose({ kind: "edit", target });
    else if (data === "u" && target.summarizeUpTo) this.actions.choose({ kind: "summarize_up_to", target });
    else if (data === "f" && target.summarizeFrom) this.actions.choose({ kind: "summarize_from", target });
  }

  render(width: number): string[] {
    const target = this.target;
    const choices =
      target === null
        ? ["nothing to select"]
        : [
            target.edit === null ? null : "e edit and resend (forks before this message)",
            target.summarizeUpTo ? "u summarize up to here" : null,
            target.summarizeFrom ? "f summarize from here" : null,
          ].filter((choice) => choice !== null);
    const position = target === null ? "" : `message ${this.index + 1} of ${this.targets.length} · `;
    return [
      truncateToWidth(muted(" ─".repeat(Math.floor(width / 2))), width),
      truncateToWidth(
        ` ${accent("↑/↓ select")} ${muted(`· ${position}${[...choices, "Esc back"].join(" · ")}`)}`,
        width,
      ),
    ];
  }

  invalidate(): void {}
}
