/**
 * Browser SpeechSynthesis wrapper — speaks the agent's replies on the phone.
 * Primary feedback channel for blind users (on by default in the UI).
 */
export function isSpeechSupported(): boolean {
  return typeof window !== "undefined" && "speechSynthesis" in window;
}

/**
 * Must be called inside a user gesture (iOS won't allow programmatic speech
 * otherwise). Speaks a silent utterance to "unlock" later speak() calls.
 */
export function unlockSpeech(): void {
  if (!isSpeechSupported()) return;
  try {
    const u = new SpeechSynthesisUtterance(" ");
    u.volume = 0;
    window.speechSynthesis.speak(u);
    window.speechSynthesis.resume();
  } catch {
    /* ignore */
  }
}

export function speak(text: string): void {
  if (!isSpeechSupported() || !text.trim()) return;
  try {
    const u = new SpeechSynthesisUtterance(text);
    u.lang = "en-US";
    u.rate = 1;
    window.speechSynthesis.speak(u);
  } catch {
    /* ignore */
  }
}

export function cancelSpeech(): void {
  if (!isSpeechSupported()) return;
  try {
    window.speechSynthesis.cancel();
  } catch {
    /* ignore */
  }
}
