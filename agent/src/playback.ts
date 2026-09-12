import {
  Image,
  Text,
  truncateToWidth,
  type Component,
  type TuiMouseEvent,
} from "@earendil-works/pi-tui";
import { readArtifact } from "./artifacts.js";
import { frameDelays, type Clip } from "./clips.js";
import { accent, clean, muted } from "./terminal-style.js";

/** A finite, source-timed image player. It never reads a recording or repeats a query. */
export class Playback implements Component {
  private images: Image[] = [];
  private timer?: ReturnType<typeof setTimeout>;
  private disposed = false;
  private error?: string;
  private scrubRow = 0;
  private index = 0;
  private running = false;
  readonly ready: Promise<void>;
  constructor(
    readonly clip: Clip,
    private readonly changed: () => void,
    private autoplay = false,
  ) {
    this.ready = this.load()
      .then(() => {
        if (this.autoplay) this.play();
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
        new Image(
          bytes.toString("base64"),
          "image/png",
          { fallbackColor: muted },
          { maxWidthCells: 120, maxHeightCells: 25 },
        ),
      );
    }
    this.changed();
  }
  play(): void {
    if (
      this.disposed ||
      this.images.length !== this.clip.frames.length ||
      this.error
    )
      return;
    clearTimeout(this.timer);
    if (this.index === this.clip.frames.length - 1) this.index = 0;
    this.running = this.clip.frames.length > 1;
    this.schedule();
    this.changed();
  }
  pause(): void {
    this.autoplay = false;
    clearTimeout(this.timer);
    this.running = false;
    if (!this.disposed) this.changed();
  }
  seek(seconds: number): void {
    if (this.disposed || !Number.isFinite(seconds)) return;
    this.pause();
    const absolute = this.clip.timeOrigin + seconds;
    this.index = Math.max(
      0,
      this.clip.frames.findLastIndex((frame) => frame.timestamp <= absolute),
    );
    this.changed();
  }
  private schedule(): void {
    if (!this.running || this.disposed) return;
    const delays = frameDelays(
      this.clip.frames.map((frame) => frame.timestamp),
    );
    this.timer = setTimeout(() => {
      if (this.disposed) return;
      if (this.index < this.clip.frames.length - 1) this.index++;
      else this.running = false;
      this.changed();
      this.schedule();
    }, delays[this.index]);
  }
  close(): void {
    this.disposed = true;
    this.running = false;
    clearTimeout(this.timer);
  }
  invalidate(): void {
    for (const image of this.images) image.invalidate();
  }
  handleMouse(
    event: TuiMouseEvent,
  ): { handled: boolean; render: boolean } | undefined {
    if (event.type !== "press" || event.button !== "left") return;
    if (event.y === 0 && event.x < 14) {
      if (this.running) this.pause();
      else this.play();
    } else if (event.y === this.scrubRow) {
      const first = this.clip.frames[0].timestamp,
        last = this.clip.frames.at(-1)!.timestamp;
      this.seek(
        first -
          this.clip.timeOrigin +
          Math.max(
            0,
            Math.min(1, (event.x - 1) / Math.max(1, event.width - 3)),
          ) *
            (last - first),
      );
    } else return;
    return { handled: true, render: true };
  }
  render(width: number): string[] {
    const frame = this.clip.frames[this.index],
      time = frame.timestamp - this.clip.timeOrigin;
    const header =
      accent(this.running ? " ⏸ Pause" : " ▶ Play") +
      muted(
        `  ${time.toFixed(3)} s · frame ${frame.index + 1}/${this.clip.sourceCount} · ${this.clip.type}`,
      );
    const lines = [
      truncateToWidth(header, width),
      ...(this.images[this.index] && !this.error
        ? this.images[this.index].render(width)
        : new Text(
            muted(
              this.error
                ? `Preview unavailable: ${this.error}`
                : "Loading saved frames…",
            ),
            0,
            0,
          ).render(width)),
    ];
    this.scrubRow = lines.length;
    const count = Math.max(1, width - 2),
      first = this.clip.frames[0].timestamp,
      duration = this.clip.frames.at(-1)!.timestamp - first;
    const head = Math.round(
      (duration ? (frame.timestamp - first) / duration : 0) * (count - 1),
    );
    lines.push(
      truncateToWidth(
        " " +
          muted("━".repeat(head)) +
          accent("●") +
          muted("─".repeat(count - head - 1)),
        width,
      ),
    );
    lines.push(
      ...new Text(
        muted(
          " /play · /pause · /seek SECONDS · click timeline · source-timed playback",
        ),
        0,
        0,
      ).render(width),
    );
    return lines;
  }
}
