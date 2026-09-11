/** Count completed draws, not simulation messages or scheduled animation callbacks. */
export class FrameRate {
  private started: number | null = null;
  private frames = 0;
  reset(): void {
    this.started = null;
    this.frames = 0;
  }
  draw(now: number): number | null {
    if (this.started === null) {
      this.started = now;
      return null;
    }
    this.frames++;
    const elapsed = now - this.started;
    if (elapsed < 1000) return null;
    const fps = this.frames * 1000 / elapsed;
    this.started = now;
    this.frames = 0;
    return fps;
  }
}
