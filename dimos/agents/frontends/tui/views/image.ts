import type { RenderedView } from "#dimos/agents/shared/protocol/media.ts";
import type { ViewPanel } from "#dimos/agents/frontends/tui/views/base.ts";
import { TerminalImage } from "#dimos/agents/frontends/tui/lib/terminal-image.ts";
import { Text } from "@earendil-works/pi-tui";
import { clean } from "#dimos/agents/shared/view-models/text.ts";
import { muted } from "#dimos/agents/frontends/tui/style.ts";
import { readVerified } from "#dimos/agents/shared/render/files.ts";

/** The cached preview of one image. */
export class ImagePanel implements ViewPanel<"image"> {
  readonly view: Extract<RenderedView, { readonly kind: "image" }>;
  private image: TerminalImage | null = null;
  private error: string | null = null;
  private disposed = false;

  constructor(view: Extract<RenderedView, { readonly kind: "image" }>, changed: () => void) {
    this.view = view;
    readVerified(view.preview).then(
      (png) => {
        if (this.disposed) return;
        this.image = new TerminalImage(png, { maxWidthCells: 120, maxHeightCells: 26 }, changed);
        changed();
      },
      (error: unknown) => {
        if (this.disposed) return;
        this.error = clean(String(error));
        changed();
      },
    );
  }

  render(width: number): string[] {
    if (this.image !== null) return this.image.render(width);
    return new Text(
      muted(this.error === null ? " Loading preview…" : ` Preview unavailable: ${this.error}`),
      0,
      0,
    ).render(width);
  }

  invalidate(): void {
    this.image?.invalidate();
  }

  dispose(): void {
    this.disposed = true;
    this.image?.close();
  }
}
