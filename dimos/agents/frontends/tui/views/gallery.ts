import type { Component, TuiMouseEvent } from "@earendil-works/pi-tui";
import type { FileRef, RenderDetails, RenderedView } from "#dimos/agents/shared/protocol/media.ts";
import type { ViewPanel } from "#dimos/agents/frontends/tui/views/base.ts";
import { ClipPanel } from "#dimos/agents/frontends/tui/views/clip.ts";
import { ImagePanel } from "#dimos/agents/frontends/tui/views/image.ts";
import { SpatialPanel } from "#dimos/agents/frontends/tui/views/points.ts";
import { TerminalImage } from "#dimos/agents/frontends/tui/lib/terminal-image.ts";
import { Text, truncateToWidth, visibleWidth } from "@earendil-works/pi-tui";
import { accent, amber, muted } from "#dimos/agents/frontends/tui/style.ts";
import { clean } from "#dimos/agents/shared/view-models/text.ts";
import { overviewOf } from "#dimos/agents/shared/render/raster.ts";
import { readVerified } from "#dimos/agents/shared/render/files.ts";

function previewOf(view: RenderedView): FileRef {
  switch (view.kind) {
    case "image":
    case "points":
      return view.preview;
    case "clip":
      return view.sheet;
  }
}

function panelFor(view: RenderedView, changed: () => void): ViewPanel<RenderedView["kind"]> {
  switch (view.kind) {
    case "image":
      return new ImagePanel(view, changed);
    case "points":
      return new SpatialPanel(view, changed);
    case "clip":
      return new ClipPanel(view, changed);
  }
}

/**
 * One rendering under a card or an event line: an overview mosaic of up to six views, a panel per view chosen by
 * click or `/panel N`, and the listed paths. Moved from dimcode tool-images.ts.
 */
export class Gallery implements Component {
  readonly details: RenderDetails;
  private readonly changed: () => void;
  private readonly openPanel: (index: number) => void;
  private overview: TerminalImage | null = null;
  private overviewError: string | null = null;
  private focused: ViewPanel<RenderedView["kind"]> | null = null;
  private selected = 0;
  private tabs: { readonly start: number; readonly end: number; readonly index: number }[] = [];
  private disposed = false;

  constructor(details: RenderDetails, changed: () => void, openPanel: (index: number) => void) {
    this.details = details;
    this.changed = changed;
    this.openPanel = openPanel;
    if (details.views.length > 0) {
      Promise.all(details.views.map((view) => readVerified(previewOf(view))))
        .then(overviewOf)
        .then(
          (png) => {
            if (this.disposed) return;
            this.overview = new TerminalImage(png, { maxWidthCells: 120, maxHeightCells: 26 }, changed);
            changed();
          },
          (error: unknown) => {
            if (this.disposed) return;
            this.overviewError = clean(String(error));
            changed();
          },
        );
    }
  }

  /** Whether the focused panel takes keys, so the terminal gives it focus. */
  get interactive(): boolean {
    return this.focused?.handleInput !== undefined;
  }

  /** Focuses one view's panel (1-based, as the tabs number them), or the overview for 0. */
  select(index: number): void {
    const view = this.details.views[index - 1];
    this.focused?.dispose();
    this.focused = view === undefined ? null : panelFor(view, this.changed);
    this.selected = view === undefined ? 0 : index;
    this.changed();
  }

  handleInput(data: string): void {
    this.focused?.handleInput?.(data);
  }

  handleMouse(event: TuiMouseEvent): { handled: boolean; render: boolean } | undefined {
    if (event.type !== "press" || event.y !== 0) return undefined;
    const tab = this.tabs.find((candidate) => event.x >= candidate.start && event.x < candidate.end);
    if (tab === undefined) return undefined;
    this.openPanel(tab.index);
    return { handled: true, render: true };
  }

  render(width: number): string[] {
    const lines: string[] = [];
    if (this.details.views.length > 0) {
      const only = this.details.views.length === 1 ? this.details.views[0] : undefined;
      lines.push(
        only?.kind === "image" ? truncateToWidth(accent(` ${clean(only.label)}`), width) : this.tabLine(width),
      );
      if (this.focused !== null) lines.push(...this.focused.render(width));
      else if (this.overview !== null) lines.push(...this.overview.render(width));
      else
        lines.push(
          ...new Text(
            muted(this.overviewError === null ? " Preparing views…" : ` Preview unavailable: ${this.overviewError}`),
            0,
            0,
          ).render(width),
        );
    }
    for (const path of this.details.listed) lines.push(truncateToWidth(muted(` Also: ${clean(path)}`), width));
    for (const failure of this.details.failed)
      lines.push(truncateToWidth(amber(` Not rendered: ${clean(failure.path)}: ${clean(failure.error)}`), width));
    if (this.details.views.length > 1) {
      lines.push(...new Text(muted(` /panel 0–${this.details.views.length} selects a view`), 0, 0).render(width));
    }
    return lines;
  }

  invalidate(): void {
    this.overview?.invalidate();
    this.focused?.invalidate();
  }

  dispose(): void {
    this.disposed = true;
    this.overview?.close();
    this.focused?.dispose();
  }

  private tabLine(width: number): string {
    const labels = ["Overview", ...this.details.views.map((view) => view.label)];
    let line = " ";
    this.tabs = [];
    for (const [index, label] of labels.entries()) {
      const text = `${index} ${clean(label).slice(0, 16)}  `;
      const start = visibleWidth(line);
      const end = start + visibleWidth(text);
      if (end > width) break;
      line += (index === this.selected ? accent : muted)(text);
      this.tabs.push({ start, end, index });
    }
    return truncateToWidth(line, width);
  }
}
