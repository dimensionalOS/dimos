import {
  truncateToWidth,
  visibleWidth,
  type Component,
} from "@earendil-works/pi-tui";

export const accent = (s: string) => `\x1b[38;2;106;229;207m${s}\x1b[39m`;
export const muted = (s: string) => `\x1b[38;2;139;164;168m${s}\x1b[39m`;
export const amber = (s: string) => `\x1b[38;2;245;192;120m${s}\x1b[39m`;
export const clean = (s: string): string =>
  s.replace(/[\x00-\x1f\x7f-\x9f]/g, " ");

/** Width-aware chrome; contents come from the active session, never placeholder metrics. */
export class StatusLine implements Component {
  constructor(private readonly content: () => [string, string?]) {}
  invalidate(): void {}
  render(width: number): string[] {
    const [left, right = ""] = this.content();
    const available = Math.max(0, width - visibleWidth(right) - 4);
    if (available < 20) return [truncateToWidth(" " + left, width)];
    const clipped = truncateToWidth(left, available);
    return [
      " " +
        clipped +
        " ".repeat(
          Math.max(1, width - visibleWidth(clipped) - visibleWidth(right) - 2),
        ) +
        right +
        " ",
    ];
  }
}
