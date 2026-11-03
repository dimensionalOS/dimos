import type { Component } from "@earendil-works/pi-tui";
import type { Level } from "#dimos/agents/shared/view-models/levels.ts";
import { clean } from "#dimos/agents/shared/view-models/text.ts";
import { truncateToWidth, visibleWidth } from "@earendil-works/pi-tui";

/** Moved as is from dimcode terminal-style.ts. */
export function accent(text: string): string {
  return `\x1b[38;2;106;229;207m${text}\x1b[39m`;
}

export function muted(text: string): string {
  return `\x1b[38;2;139;164;168m${text}\x1b[39m`;
}

export function amber(text: string): string {
  return `\x1b[38;2;245;192;120m${text}\x1b[39m`;
}

export function red(text: string): string {
  return `\x1b[38;2;240;113;120m${text}\x1b[39m`;
}

/** A datum's level as its colour: plain when it needs nobody, amber for a look, red for now. */
export function levelColor(level: Level, text: string): string {
  switch (level) {
    case "ok":
      return clean(text);
    case "warn":
      return amber(clean(text));
    case "critical":
      return red(clean(text));
  }
}

/** A width-aware one-line status: `content()` gives the left text and an optional right one, read at each render. */
export class StatusLine implements Component {
  private readonly content: () => readonly [string, string?];

  constructor(content: () => readonly [string, string?]) {
    this.content = content;
  }

  invalidate(): void {}

  render(width: number): string[] {
    const [left, right = ""] = this.content();
    const available = Math.max(0, width - visibleWidth(right) - 4);
    if (available < 20) return [truncateToWidth(` ${left}`, width)];
    const clipped = truncateToWidth(left, available);
    return [` ${clipped}${" ".repeat(Math.max(1, width - visibleWidth(clipped) - visibleWidth(right) - 2))}${right} `];
  }
}
