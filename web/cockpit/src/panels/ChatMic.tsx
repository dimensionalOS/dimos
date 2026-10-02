// Push-to-talk mic for the chat composer. Hold to record, release to send:
// the recording ships as audio.json.v1 chunks on the chat panel's audio
// channel, and the robot-side VoiceInput module transcribes it onto
// /human_input - the utterance then appears in the transcript when the
// agent echoes it, exactly like typed text. Renders nothing when the
// microphone APIs are unavailable (insecure context, no MediaRecorder).
// Element-scoped listeners only: holding the mic never drives the robot.

import { useEffect, useMemo, useSyncExternalStore } from "react";
import type { Session } from "@dimos/sdk";
import { type RecorderPort, VoiceRecorder } from "./voiceRecorder.ts";
import styles from "./ChatPanel.module.css";

// Whatever the browser records is shipped as-is (mime rides every frame);
// the preference order just picks the smallest container Chrome/Firefox
// offer before Safari's mp4.
const MIME_CANDIDATES = ["audio/webm;codecs=opus", "audio/webm", "audio/mp4"];
const AUDIO_BITS_PER_SECOND = 32_000;

function supported(): boolean {
  return globalThis.isSecureContext === true &&
    typeof navigator !== "undefined" &&
    typeof navigator.mediaDevices?.getUserMedia === "function" &&
    "MediaRecorder" in globalThis;
}

/** A MediaRecorder on the shared mic stream, adapted to the machine's port. */
async function openRecorder(
  mic: { current: MediaStream | null },
  ondata: (bytes: Uint8Array) => void,
): Promise<RecorderPort> {
  if (mic.current === null) {
    mic.current = await navigator.mediaDevices.getUserMedia({ audio: true });
  }
  const mimeType = typeof MediaRecorder.isTypeSupported === "function"
    ? MIME_CANDIDATES.find((m) => MediaRecorder.isTypeSupported(m))
    : undefined;
  const recorder = new MediaRecorder(mic.current, {
    ...(mimeType === undefined ? {} : { mimeType }),
    audioBitsPerSecond: AUDIO_BITS_PER_SECOND,
  });
  // Blob -> bytes is async; the chain keeps delivery in recording order.
  // stop() settles only after the last delivery - resolving on success,
  // rejecting when capture or a Blob read failed, so a broken recording
  // surfaces as the machine's error phase instead of wedging in `sending`.
  let failure: Error | null = null;
  const fail = (err: unknown): void => {
    failure ??= err instanceof Error ? err : new Error(String(err));
  };
  let deliveries = Promise.resolve();
  let settle: () => void = () => {};
  const lastData = new Promise<void>((resolve, reject) => {
    settle = () => (failure === null ? resolve() : reject(failure));
  });
  recorder.ondataavailable = (event) => {
    deliveries = deliveries.then(async () => {
      try {
        const bytes = new Uint8Array(await event.data.arrayBuffer());
        if (failure === null && bytes.length > 0) ondata(bytes);
      } catch (err) {
        fail(err);
      }
    });
  };
  recorder.onerror = (event) => {
    fail((event as { error?: unknown }).error ?? new Error("recorder failed"));
  };
  recorder.onstop = () => void deliveries.then(settle);
  return {
    mimeType: recorder.mimeType || mimeType || "audio/webm",
    start: (timesliceMs) => recorder.start(timesliceMs),
    stop: () => {
      if (recorder.state === "inactive") void deliveries.then(settle);
      else recorder.stop();
      return lastData;
    },
  };
}

export interface ChatMicProps {
  session: Session;
  ch: string;
  connected: boolean;
  /** Errors land in the chat panel's existing alert row; null clears. */
  onError: (message: string | null) => void;
}

export function ChatMic(props: ChatMicProps) {
  if (!supported()) return null;
  return <Mic {...props} />;
}

function Mic({ session, ch, connected, onError }: ChatMicProps) {
  const recorder = useMemo(() => {
    // The mic stream stays open between holds: press-to-speak must not lose
    // the first words to a getUserMedia round-trip. The cost is the OS
    // record indicator while the panel lives; unmount releases it.
    const mic = { current: null as MediaStream | null };
    return {
      mic,
      machine: new VoiceRecorder({
        openMic: (ondata) => openRecorder(mic, ondata),
        publish: (frame) => session.publish(ch, frame),
      }),
    };
  }, [session, ch]);
  const state = useSyncExternalStore(recorder.machine.subscribe, recorder.machine.getSnapshot);
  useEffect(() => () => {
    recorder.machine.cancel();
    recorder.mic.current?.getTracks().forEach((track) => track.stop());
    recorder.mic.current = null;
  }, [recorder]);
  useEffect(() => {
    if (state.phase === "error") onError(`voice failed: ${state.message}`);
    else if (state.phase === "arming") onError(null);
  }, [state, onError]);

  const held = state.phase === "arming" || state.phase === "recording";
  return (
    <button
      type="button"
      className={styles.mic}
      data-state={state.phase}
      data-testid={`chat-${ch}-mic`}
      aria-label="hold to talk"
      // Never disable mid-hold: a disabled button swallows the pointerup
      // that must end the recording.
      disabled={!connected && !held && state.phase !== "sending"}
      onPointerDown={(e) => {
        // Capture keeps the release on the button even if the pointer
        // drifts off it mid-hold (guarded: synthetic events lack ids).
        if (typeof e.currentTarget.setPointerCapture === "function" && e.pointerId >= 0) {
          e.currentTarget.setPointerCapture(e.pointerId);
        }
        recorder.machine.press();
      }}
      onPointerUp={() => recorder.machine.release()}
      onPointerCancel={() => recorder.machine.cancel()}
      onKeyDown={(e) => {
        if (e.key === "Escape") recorder.machine.cancel();
      }}
    >
      {state.phase === "recording" ? "listening" : state.phase === "sending" ? "sending" : "talk"}
    </button>
  );
}
