/** Wrap a transcript so the agent knows it came from voice (WEBAPP-BRIEF §5). */
export function wrapUserSpeech(text: string): string {
  return `<user_speech>${text.trim()}</user_speech>`;
}

export interface SttCallbacks {
  /** live partial transcript (Web Speech only) */
  onInterim?: (text: string) => void;
  /** final transcript to submit */
  onFinal: (text: string) => void;
  onError?: (err: unknown) => void;
}

export interface SttProvider {
  readonly mode: "webspeech" | "upload";
  isSupported(): boolean;
  start(cb: SttCallbacks): void | Promise<void>;
  stop(): void;
}

/* ---- minimal Web Speech API typings (not in lib.dom for all targets) ---- */
interface SpeechRecognitionAlternativeLike {
  transcript: string;
}
interface SpeechRecognitionResultLike
  extends ArrayLike<SpeechRecognitionAlternativeLike> {
  isFinal: boolean;
}
interface SpeechRecognitionEventLike {
  results: ArrayLike<SpeechRecognitionResultLike>;
}
/** onerror payload — `.error` is a code like "no-speech" | "network" | "not-allowed". */
export interface SpeechRecognitionErrorLike {
  error: string;
  message?: string;
}
interface SpeechRecognitionLike {
  lang: string;
  continuous: boolean;
  interimResults: boolean;
  onresult: ((e: SpeechRecognitionEventLike) => void) | null;
  onerror: ((e: SpeechRecognitionErrorLike) => void) | null;
  onend: (() => void) | null;
  start(): void;
  stop(): void;
  abort(): void;
}
type SpeechRecognitionCtor = new () => SpeechRecognitionLike;

declare global {
  interface Window {
    SpeechRecognition?: SpeechRecognitionCtor;
    webkitSpeechRecognition?: SpeechRecognitionCtor;
  }
}

/** Live, on-device transcription. The active provider for the design-first build. */
class WebSpeechStt implements SttProvider {
  readonly mode = "webspeech" as const;
  private rec: SpeechRecognitionLike | null = null;
  private latest = "";

  private ctor(): SpeechRecognitionCtor | undefined {
    if (typeof window === "undefined") return undefined;
    return window.SpeechRecognition ?? window.webkitSpeechRecognition;
  }

  isSupported() {
    return !!this.ctor();
  }

  /**
   * Detach handlers and abort any live session so it can't keep the mic open
   * or fire stale callbacks. Prevents overlapping sessions when the button is
   * pressed again before the previous recognition has fully ended.
   */
  private teardown() {
    const rec = this.rec;
    if (!rec) return;
    this.rec = null;
    rec.onresult = null;
    rec.onerror = null;
    rec.onend = null;
    try {
      rec.abort();
    } catch {
      /* already stopped */
    }
  }

  start(cb: SttCallbacks) {
    const Ctor = this.ctor();
    if (!Ctor) {
      cb.onError?.(new Error("SpeechRecognition unsupported"));
      return;
    }
    this.teardown(); // replace any previous session cleanly

    const rec = new Ctor();
    this.rec = rec;
    this.latest = "";
    rec.lang = "en-US";
    rec.continuous = true;
    rec.interimResults = true;
    rec.onresult = (e) => {
      let full = "";
      for (let i = 0; i < e.results.length; i++) {
        full += e.results[i][0]?.transcript ?? "";
      }
      this.latest = full;
      cb.onInterim?.(full.trim());
    };
    rec.onerror = (ev) => cb.onError?.(ev);
    rec.onend = () => {
      if (this.rec !== rec) return; // superseded by a newer session — ignore
      this.rec = null;
      cb.onFinal(this.latest.trim());
    };
    try {
      rec.start();
    } catch (e) {
      // e.g. iOS throws if a prior session is still tearing down
      this.teardown();
      cb.onError?.(e);
    }
  }

  stop() {
    try {
      this.rec?.stop();
    } catch {
      /* not running */
    }
  }
}

/**
 * Production path: record audio/mp4 and POST to /upload_audio for transcription.
 * Written and ready; enable with NEXT_PUBLIC_STT=upload for the real iPhone demo.
 */
class UploadStt implements SttProvider {
  readonly mode = "upload" as const;
  private mr: MediaRecorder | null = null;
  private stream: MediaStream | null = null;
  private chunks: Blob[] = [];

  constructor(
    private upload: (blob: Blob, filename?: string) => Promise<string>,
  ) {}

  isSupported() {
    return (
      typeof window !== "undefined" &&
      typeof MediaRecorder !== "undefined" &&
      !!navigator.mediaDevices?.getUserMedia
    );
  }

  async start(cb: SttCallbacks) {
    try {
      this.stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    } catch (e) {
      cb.onError?.(e);
      return;
    }
    // iOS Safari requires audio/mp4, not audio/webm (WEBAPP-BRIEF §6).
    const mime = MediaRecorder.isTypeSupported("audio/mp4")
      ? "audio/mp4"
      : "audio/webm";
    const mr = new MediaRecorder(this.stream, { mimeType: mime });
    this.mr = mr;
    this.chunks = [];
    mr.ondataavailable = (e) => {
      if (e.data.size) this.chunks.push(e.data);
    };
    mr.onstop = async () => {
      this.stream?.getTracks().forEach((t) => t.stop());
      const blob = new Blob(this.chunks, { type: mime });
      try {
        const text = await this.upload(
          blob,
          mime.includes("mp4") ? "recording.mp4" : "recording.webm",
        );
        cb.onFinal(text.trim());
      } catch (e) {
        cb.onError?.(e);
      }
    };
    cb.onInterim?.("…");
    mr.start();
  }

  stop() {
    if (this.mr && this.mr.state !== "inactive") this.mr.stop();
  }
}

/** Pick the provider from NEXT_PUBLIC_STT (default: webspeech). */
export function createSttProvider(
  upload: (blob: Blob, filename?: string) => Promise<string>,
): SttProvider {
  const mode = process.env.NEXT_PUBLIC_STT ?? "webspeech";
  return mode === "upload" ? new UploadStt(upload) : new WebSpeechStt();
}
