import { useCallback, useRef, useState } from "react";
import { createSttProvider, type SttProvider } from "@/lib/stt";
import * as dimos from "@/lib/dimos";

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
          setError("Couldn't capture speech — check mic permission.");
          console.error("stt error", e);
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
