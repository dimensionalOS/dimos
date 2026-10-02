// Push-to-talk recorder: the chat mic's press/hold/release/cancel state
// machine and chunk sender, free of DOM and MediaRecorder so tests drive it
// with fakes (ChatMic adapts the real APIs).
//
// One button hold is one utterance (a fresh sid): container bytes stream in
// through ondata while recording, are sliced to <=16 KiB and published in
// order as audio.json.v1 frames, and a final empty frame closes the
// utterance after release. Each send awaits the previous ack and a 60 ms
// floor, staying inside the channel's 20 Hz relay token buckets (capacity
// 20, refill 20/s). Cancel drops everything unsent and sends no final frame;
// the robot reaps the half-built utterance instead.

/** One audio.json.v1 frame (a type alias, so it assigns to JsonValue). */
export type AudioFrame = {
  sid: string;
  seq: number;
  mime: string;
  data: string;
  final: boolean;
};

export type VoicePhase = "idle" | "arming" | "recording" | "sending" | "error";

export interface VoiceSnapshot {
  phase: VoicePhase;
  /** Set in the error phase (a PublishError message reads "code: reason"). */
  message?: string;
}

export interface RecorderPort {
  mimeType: string;
  start(timesliceMs: number): void;
  /** Stops capture; resolves once every recorded byte has reached ondata. */
  stop(): Promise<void>;
}

export interface VoiceRecorderDeps {
  /** Opens the mic; ondata receives the container byte stream in order. */
  openMic(ondata: (bytes: Uint8Array) => void): Promise<RecorderPort>;
  publish(frame: AudioFrame): Promise<unknown>;
  newSid?(): string;
  sleep?(ms: number): Promise<void>;
}

export const TIMESLICE_MS = 500;
export const MAX_SLICE_BYTES = 16 * 1024;
export const MIN_SEND_INTERVAL_MS = 60;
export const MAX_UTTERANCE_MS = 90_000;

function b64(bytes: Uint8Array): string {
  let s = "";
  for (const b of bytes) s += String.fromCharCode(b);
  return btoa(s);
}

function messageOf(err: unknown): string {
  return err instanceof Error ? err.message : String(err);
}

interface Utterance {
  sid: string;
  seq: number;
  mime: string;
  port: RecorderPort | null;
  queue: Uint8Array[];
  released: boolean;
  cancelled: boolean;
  kick: () => void;
}

export class VoiceRecorder {
  #openMic: VoiceRecorderDeps["openMic"];
  #publish: VoiceRecorderDeps["publish"];
  #newSid: () => string;
  #sleep: (ms: number) => Promise<void>;
  #snapshot: VoiceSnapshot = { phase: "idle" };
  #listeners = new Set<() => void>();
  #current: Utterance | null = null;

  constructor(deps: VoiceRecorderDeps) {
    this.#openMic = deps.openMic;
    this.#publish = deps.publish;
    this.#newSid = deps.newSid ?? (() => crypto.randomUUID());
    this.#sleep = deps.sleep ?? ((ms) => new Promise((resolve) => setTimeout(resolve, ms)));
  }

  /** useSyncExternalStore pair; the snapshot identity changes per transition. */
  subscribe = (cb: () => void): () => void => {
    this.#listeners.add(cb);
    return () => this.#listeners.delete(cb);
  };
  getSnapshot = (): VoiceSnapshot => this.#snapshot;

  /** Button pressed: start an utterance (also clears a shown error). */
  press(): void {
    if (this.#snapshot.phase !== "idle" && this.#snapshot.phase !== "error") return;
    const utterance: Utterance = {
      sid: this.#newSid(),
      seq: 0,
      mime: "",
      port: null,
      queue: [],
      released: false,
      cancelled: false,
      kick: () => {},
    };
    this.#current = utterance;
    this.#set({ phase: "arming" });
    void this.#run(utterance);
  }

  /** Button released: flush what was recorded and close with a final frame. */
  release(): void {
    const u = this.#current;
    if (u === null || u.released || u.cancelled) return;
    u.released = true;
    u.kick();
  }

  /** Escape / pointer cancel: drop the utterance; nothing more is sent. */
  cancel(): void {
    const u = this.#current;
    if (u === null || u.cancelled) return;
    u.cancelled = true;
    u.kick();
    u.port?.stop().catch(() => {});
    this.#finish(u, { phase: "idle" });
  }

  #set(snapshot: VoiceSnapshot): void {
    this.#snapshot = snapshot;
    for (const cb of this.#listeners) cb();
  }

  #setFor(u: Utterance, snapshot: VoiceSnapshot): void {
    if (this.#current === u) this.#set(snapshot);
  }

  #finish(u: Utterance, snapshot: VoiceSnapshot): void {
    if (this.#current !== u) return;
    this.#current = null;
    this.#set(snapshot);
  }

  async #run(u: Utterance): Promise<void> {
    let port: RecorderPort;
    try {
      port = await this.#openMic((bytes) => {
        for (let i = 0; i < bytes.length; i += MAX_SLICE_BYTES) {
          u.queue.push(bytes.subarray(i, i + MAX_SLICE_BYTES));
        }
        u.kick();
      });
    } catch (err) {
      this.#finish(u, { phase: "error", message: `microphone unavailable: ${messageOf(err)}` });
      return;
    }
    u.port = port;
    u.mime = port.mimeType;
    if (u.cancelled) {
      port.stop().catch(() => {});
      return;
    }
    if (u.released) {
      // Released before capture began: nothing recorded, nothing sent.
      port.stop().catch(() => {});
      this.#finish(u, { phase: "idle" });
      return;
    }
    port.start(TIMESLICE_MS);
    this.#setFor(u, { phase: "recording" });
    void this.#autoRelease(u);
    try {
      await this.#pump(u);
    } catch (err) {
      if (!u.cancelled) {
        u.cancelled = true;
        port.stop().catch(() => {});
        this.#finish(u, { phase: "error", message: messageOf(err) });
      }
      return;
    }
    this.#finish(u, { phase: "idle" });
  }

  async #pump(u: Utterance): Promise<void> {
    let flushed = false;
    while (true) {
      if (u.cancelled) return;
      if (u.released && !flushed) {
        this.#setFor(u, { phase: "sending" });
        // Resolves after the recorder's last bytes landed in the queue.
        await u.port!.stop();
        flushed = true;
        continue;
      }
      const piece = u.queue.shift();
      if (piece !== undefined) {
        await this.#send(u, b64(piece), false);
        continue;
      }
      if (flushed) break;
      // Install the waker before re-checking, so a byte arriving between
      // the empty shift() above and this point cannot be a lost wakeup.
      const wake = new Promise<void>((resolve) => {
        u.kick = resolve;
      });
      if (u.queue.length > 0 || u.released || u.cancelled) continue;
      await wake;
    }
    if (u.cancelled) return;
    await this.#send(u, "", true);
  }

  async #send(u: Utterance, data: string, final: boolean): Promise<void> {
    const frame: AudioFrame = { sid: u.sid, seq: u.seq++, mime: u.mime, data, final };
    // The floor runs concurrently with the ack wait: the next send starts
    // after both, spacing frames >= MIN_SEND_INTERVAL_MS apart.
    await Promise.all([
      this.#publish(frame),
      final ? Promise.resolve() : this.#sleep(MIN_SEND_INTERVAL_MS),
    ]);
  }

  async #autoRelease(u: Utterance): Promise<void> {
    await this.#sleep(MAX_UTTERANCE_MS);
    if (this.#current === u && !u.released && !u.cancelled) this.release();
  }
}
