import type { RenderedView } from "../../../shared/protocol/media.ts";
import type { ViewPanel } from "./base.ts";
import { TerminalImage } from "../lib/terminal-image.ts";
import { Text, getCapabilities } from "@earendil-works/pi-tui";
import { clean } from "../../../shared/view-models/text.ts";
import { muted } from "../style.ts";
import { readVerified } from "../../../shared/render/files.ts";

type ClipView = Extract<RenderedView, { readonly kind: "clip" }>;

/** How long each frame stays up: the gap to the next frame's timestamp, 100 ms after the last, at least 10 ms. */
function frameDelays(times: readonly number[]): number[] {
  return times.map((time, index) => Math.max(10, Math.round(((times[index + 1] ?? time + 0.1) - time) * 1000)));
}

/** Loops a clip's cached frame PNGs at source timing; never plays the GIF. Moved from dimcode playback.ts. */
export class ClipPanel implements ViewPanel<"clip"> {
  readonly view: ClipView;
  private readonly changed: () => void;
  private readonly delays: readonly number[];
  private readonly frames: Buffer[] = [];
  private readonly nativeImages = new Map<number, TerminalImage>();
  private image: TerminalImage | null = null;
  private imageIndex = 0;
  private timer: NodeJS.Timeout | null = null;
  private index = 0;
  private error: string | null = null;
  private disposed = false;

  constructor(view: ClipView, changed: () => void) {
    this.view = view;
    this.changed = changed;
    this.delays = frameDelays(view.clip.frames.map((frame) => frame.timestamp));
    this.load().catch((error: unknown) => {
      if (this.disposed) return;
      this.error = clean(String(error));
      changed();
    });
  }

  render(width: number): string[] {
    if (this.image !== null && this.error === null) {
      if (getCapabilities().images !== null) {
        let image = this.nativeImages.get(this.index);
        if (image === undefined) {
          image = new TerminalImage(this.frames[this.index]!, { maxWidthCells: 120, maxHeightCells: 25 }, this.changed);
          this.nativeImages.set(this.index, image);
        }
        return image.render(width);
      }
      if (this.imageIndex !== this.index) {
        this.image.setPng(this.frames[this.index]!);
        this.imageIndex = this.index;
      }
      return this.image.render(width);
    }
    return new Text(
      muted(this.error === null ? " Loading saved frames…" : ` Preview unavailable: ${this.error}`),
      0,
      0,
    ).render(width);
  }

  invalidate(): void {
    this.image?.invalidate();
    for (const image of this.nativeImages.values()) image.invalidate();
  }

  dispose(): void {
    this.disposed = true;
    if (this.timer !== null) clearTimeout(this.timer);
    this.image?.close();
    for (const image of this.nativeImages.values()) image.close();
  }

  private async load(): Promise<void> {
    for (const frame of this.view.clip.frames) {
      const png = await readVerified(frame.png);
      if (this.disposed) return;
      this.frames.push(png);
    }
    if (this.frames.length > 0)
      this.image = new TerminalImage(this.frames[0]!, { maxWidthCells: 120, maxHeightCells: 25 }, this.changed);
    this.changed();
    if (this.frames.length > 1) this.schedule();
  }

  private schedule(): void {
    this.timer = setTimeout(() => {
      if (this.disposed) return;
      this.index = (this.index + 1) % this.frames.length;
      this.changed();
      this.schedule();
    }, this.delays[this.index] ?? 100);
  }
}
