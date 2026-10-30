import type { Component, TUI } from "@earendil-works/pi-tui";
import type { RenderProgress } from "../../../shared/render/types.ts";
import { Loader, truncateToWidth } from "@earendil-works/pi-tui";
import { TerminalImage } from "../lib/terminal-image.ts";
import { accent, muted } from "../style.ts";
import { clean } from "../../../shared/view-models/text.ts";
import { readVerified } from "../../../shared/render/files.ts";

/** A loader, a progress bar and the latest finished frame while `/inspect` renders a clip. Moved from dimcode render-progress.ts. */
export class ProgressPanel implements Component {
  private readonly tui: TUI;
  private readonly loader: Loader;
  private progress: RenderProgress = { done: 0, total: 1, latest: null };
  private image: TerminalImage | null = null;
  private version = 0;
  private error: string | null = null;
  private closed = false;

  constructor(tui: TUI) {
    this.tui = tui;
    this.loader = new Loader(tui, accent, muted, "Rendering source frames");
  }

  update(progress: RenderProgress): void {
    if (this.closed) return;
    this.progress = progress;
    this.tui.requestRender();
    const latest = progress.latest;
    if (latest === null) return;
    const version = ++this.version;
    readVerified(latest).then(
      (png) => {
        if (this.closed || version !== this.version) return;
        if (this.image === null)
          this.image = new TerminalImage(png, { maxWidthCells: 120, maxHeightCells: 25 }, () =>
            this.tui.requestRender(),
          );
        else this.image.setPng(png);
        this.error = null;
        this.tui.requestRender();
      },
      (error: unknown) => {
        if (this.closed || version !== this.version) return;
        this.error = clean(String(error));
        this.tui.requestRender();
      },
    );
  }

  render(width: number): string[] {
    const { done, total } = this.progress;
    const size = Math.max(1, Math.min(32, width - 24));
    const filled = Math.min(size, Math.floor((size * done) / Math.max(1, total)));
    return [
      ...(this.image?.render(width) ?? []),
      ...this.loader.render(width),
      truncateToWidth(
        ` ${accent("━".repeat(filled))}${muted("─".repeat(size - filled))}${muted(` ${done}/${total}`)}`,
        width,
      ),
      ...(this.error === null ? [] : [truncateToWidth(muted(`Preview unavailable: ${this.error}`), width)]),
    ];
  }

  invalidate(): void {
    this.loader.invalidate();
    this.image?.invalidate();
  }

  dispose(): void {
    this.closed = true;
    this.loader.stop();
    this.image?.close();
  }
}
