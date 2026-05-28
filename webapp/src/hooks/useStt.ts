import { useCallback, useRef, useState } from "react";
import { createSttProvider, type SttProvider } from "@/lib/stt";
import { devLog } from "@/lib/devlog";
import * as dimos from "@/lib/dimos";

/** Map a SpeechRecognition error code to a user-facing message, or null to stay silent. */
function sttErrorMessage(code: string): string | null {
  switch (code) {
    case "no-speech":
      return "Didn't catch that — try again.";
    case "aborted":
      return null; // user released / interrupted — not an error
    case "not-allowed":
    case "service-not-allowed":
      return "Microphone blocked — allow mic access for this site.";
    case "audio-capture":
      return "No microphone found.";
    case "network":
      return "Network hiccup — try again.";
    default:
      return "Couldn't capture speech — try again.";
  }
}

/**
 * Hold-to-speak controller. `onResult` fires with the final transcript when
 * recording ends (used to submit the query).
 */
export function useStt(onResult: (finalText: string) => void) {
  const [recording, setRecording] = useState(false);
  const [transcript, setTranscript] = useState("");
  const [error, setError] = useState<string | null>(null);
  const providerRef = useRef<SttProvider | null>(null);

  const start = useCallback(() => {
    if (recording) return;
    const provider =
      providerRef.current ??
      (providerRef.current = createSttProvider(dimos.uploadAudio));
    if (!provider.isSupported()) {
      setError("Speech input isn't supported in this browser.");
      return;
    }
    setError(null);
    setTranscript("");
    setRecording(true);
    Promise.resolve(
      provider.start({
        onInterim: (t) => setTranscript(t),
        onFinal: (t) => {
          setRecording(false);
          setTranscript(t);
          if (t.trim()) onResult(t.trim());
        },
        onError: (e) => {
          setRecording(false);
          const code =
            e && typeof e === "object" && "error" in e
              ? String((e as { error: unknown }).error)
              : "unknown";
          devLog({ event: "stt-error", code });
          setError(sttErrorMessage(code)); // null for no-speech-release noise
        },
      }),
    ).catch((e) => {
      setRecording(false);
      setError("Couldn't start recording.");
      console.error("stt start failed", e);
    });
  }, [recording, onResult]);

  const stop = useCallback(() => {
    if (!recording) return;
    providerRef.current?.stop();
    // onFinal flips `recording` to false
  }, [recording]);

  return { recording, transcript, error, start, stop };
}
