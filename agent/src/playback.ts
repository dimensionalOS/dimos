import { Text, type Component } from "@earendil-works/pi-tui";
import { readArtifact } from "./artifacts.js";
import { frameDelays, type Clip } from "./clips.js";
import { clean, muted } from "./terminal-style.js";
import { TerminalImage } from "./terminal-image.js";

/** A looping presentation of saved frames. Frame selection belongs to the agent's tools. */
export class Playback implements Component {
  private images: TerminalImage[] = [];
  private timer?: ReturnType<typeof setTimeout>;
  private disposed = false;
  private error?: string;
  private index = 0;
  private running = false;
  readonly ready: Promise<void>;
  constructor(
    readonly clip: Clip,
    private readonly changed: () => void,
    private autoplay = true,
  ) {
    this.ready = this.load()
      .then(() => {
        if (this.autoplay && !this.disposed && this.images.length > 1) {
          this.running = true;
          this.schedule();
        }
      })
      .catch((error) => {
        if (!this.disposed) {
          this.error = clean(String(error));
          this.changed();
        }
      });
  }
  get frameIndex(): number {
    return this.index;
  }
  get playing(): boolean {
    return this.running;
  }
  private async load(): Promise<void> {
    for (const frame of this.clip.frames) {
      if (this.disposed) return;
      const { bytes } = await readArtifact(frame.path, frame.sha256);
      if (this.disposed) return;
      this.images.push(
        new TerminalImage(
          bytes.toString("base64"),
          "image/png",
          { fallbackColor: muted },
          { maxWidthCells: 120, maxHeightCells: 25 },
          this.changed,
        ),
      );
    }
    this.changed();
  }
  pause(): void {
    this.autoplay = false;
    this.running = false;
    clearTimeout(this.timer);
  }
  private schedule(): void {
    const delay = frameDelays(this.clip.frames.map((frame) => frame.timestamp))[
      this.index
    ];
    this.timer = setTimeout(() => {
      if (!this.running || this.disposed) return;
      this.index = (this.index + 1) % this.images.length;
      this.changed();
      this.schedule();
    }, delay);
  }
  close(): void {
    this.disposed = true;
    this.pause();
    for (const image of this.images) image.close();
  }
  invalidate(): void {
    for (const image of this.images) image.invalidate();
  }
  render(width: number): string[] {
    return this.images[this.index] && !this.error
      ? this.images[this.index].render(width)
      : new Text(
          muted(
            this.error
              ? `Preview unavailable: ${this.error}`
              : "Loading saved frames…",
          ),
          0,
          0,
        ).render(width);
  }
}
