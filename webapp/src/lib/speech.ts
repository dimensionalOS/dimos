/**
 * Speaks the agent's replies on the phone via OpenAI TTS (/api/tts) played
 * through a single <audio> element. Primary feedback channel for blind users.
 *
 * Why not the browser SpeechSynthesis API: iOS silently drops speech that isn't
 * started inside a user gesture, and agent replies arrive asynchronously over
 * SSE — so they never spoke. An <audio> element instead can be "unlocked" by
 * one tap (unlockSpeech) and then play network audio asynchronously, which iOS
 * allows. unlockSpeech MUST run synchronously inside a tap handler.
 */

let audioEl: HTMLAudioElement | null = null;

// Pending utterances, played one at a time so replies don't cut each other off.
let queue: string[] = [];
let playing = false;
let inflight: AbortController | null = null;
let endCurrent: (() => void) | null = null;

export function isSpeechSupported(): boolean {
  return typeof window !== "undefined" && typeof Audio !== "undefined";
}

function getAudio(): HTMLAudioElement {
  if (!audioEl) {
    audioEl = new Audio();
    audioEl.preload = "auto";
  }
  return audioEl;
}

/** A short, valid, silent WAV data URI — played in-gesture to unlock iOS audio. */
let silenceUri: string | null = null;
function silentWav(ms = 60, sampleRate = 8000): string {
  if (silenceUri) return silenceUri;
  const samples = Math.floor((sampleRate * ms) / 1000);
  const dataSize = samples; // 8-bit mono
  const buf = new ArrayBuffer(44 + dataSize);
  const view = new DataView(buf);
  const str = (off: number, s: string) => {
    for (let i = 0; i < s.length; i++) view.setUint8(off + i, s.charCodeAt(i));
  };
  str(0, "RIFF");
  view.setUint32(4, 36 + dataSize, true);
  str(8, "WAVE");
  str(12, "fmt ");
  view.setUint32(16, 16, true); // PCM header size
  view.setUint16(20, 1, true); // PCM
  view.setUint16(22, 1, true); // mono
  view.setUint32(24, sampleRate, true);
  view.setUint32(28, sampleRate, true); // byte rate (1 byte/sample)
  view.setUint16(32, 1, true); // block align
  view.setUint16(34, 8, true); // bits per sample
  str(36, "data");
  view.setUint32(40, dataSize, true);
  const bytes = new Uint8Array(buf);
  for (let i = 0; i < dataSize; i++) bytes[44 + i] = 128; // 8-bit silence
  let binary = "";
  for (let i = 0; i < bytes.length; i++) binary += String.fromCharCode(bytes[i]);
  silenceUri = "data:audio/wav;base64," + btoa(binary);
  return silenceUri;
}

/**
 * Must be called synchronously inside a user gesture (tap). Plays a brief
 * silent clip to unlock the <audio> element so later async play() calls (the
 * agent's replies) produce sound on iOS.
 */
export function unlockSpeech(): void {
  if (!isSpeechSupported()) return;
  try {
    const a = getAudio();
    a.src = silentWav();
    const p = a.play();
    if (p && typeof p.then === "function") p.then(() => {}, () => {});
  } catch {
    /* ignore */
  }
}

export function speak(text: string): void {
  if (!isSpeechSupported() || !text.trim()) return;
  queue.push(text.trim());
  void pump();
}

async function pump(): Promise<void> {
  if (playing) return;
  const text = queue.shift();
  if (text === undefined) return;
  playing = true;

  try {
    const ac = new AbortController();
    inflight = ac;
    const res = await fetch("/api/tts", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ text }),
      signal: ac.signal,
    });
    inflight = null;
    if (!res.ok) throw new Error(`tts ${res.status}`);

    const url = URL.createObjectURL(await res.blob());
    const a = getAudio();
    a.src = url;

    await new Promise<void>((resolve) => {
      const finish = () => {
        endCurrent = null;
        a.onended = null;
        a.onerror = null;
        resolve();
      };
      endCurrent = finish;
      a.onended = finish;
      a.onerror = finish;
      const p = a.play();
      if (p && typeof p.then === "function") p.then(() => {}, finish);
    });

    URL.revokeObjectURL(url);
  } catch {
    inflight = null;
    /* aborted or failed — fall through to the next item */
  } finally {
    playing = false;
    void pump();
  }
}

export function cancelSpeech(): void {
  queue = [];
  inflight?.abort();
  inflight = null;
  endCurrent?.(); // resolve the in-progress pump so it stops cleanly
  if (audioEl) {
    try {
      audioEl.pause();
      audioEl.removeAttribute("src");
      audioEl.load();
    } catch {
      /* ignore */
    }
  }
}
