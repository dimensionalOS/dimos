import { useCallback, useEffect, useRef, useState } from "react";
import Head from "next/head";
import type { Mode, MoveCommand, QuickAction } from "@/lib/types";
import type { AgentMessage } from "@/lib/agentMessage";
import * as dimos from "@/lib/dimos";
import { wrapUserSpeech } from "@/lib/stt";
import { devLog } from "@/lib/devlog";
import { speak, cancelSpeech, unlockSpeech } from "@/lib/speech";
import { useAgentFeed } from "@/hooks/useAgentFeed";
import { useStt } from "@/hooks/useStt";
import { useStatus } from "@/hooks/useStatus";
import { useTeleop } from "@/hooks/useTeleop";
import Header from "@/components/Header";
import ModeToggle from "@/components/ModeToggle";
import VoicePanel from "@/components/VoicePanel";
import ManualPanel from "@/components/ManualPanel";

const ZERO: MoveCommand = { vx: 0, vy: 0, turn: 0 };

export default function Home() {
  const [mode, setMode] = useState<Mode>("voice");

  // Speak the agent's replies on the phone (primary feedback for blind users).
  const [ttsEnabled, setTtsEnabled] = useState(true);
  const ttsEnabledRef = useRef(true);
  useEffect(() => {
    ttsEnabledRef.current = ttsEnabled;
  }, [ttsEnabled]);

  // Barge-in: while the user is holding to speak, suppress incoming TTS so the
  // previous turn's reply doesn't talk over them. Synced from `recording` below.
  const recordingRef = useRef(false);

  const handleAgentMessage = useCallback((m: AgentMessage) => {
    // Speak only final `ai` replies, and not while the user is mid-utterance.
    const willSpeak =
      ttsEnabledRef.current && m.kind === "ai" && !recordingRef.current;
    devLog({ event: "tts", spoke: willSpeak, kind: m.kind, text: m.text });
    if (willSpeak) speak(m.text);
  }, []);

  const { messages, active, markIdle } = useAgentFeed({
    onMessage: handleAgentMessage,
  });
  const teleop = useTeleop();
  const [drive, setDrive] = useState<MoveCommand>(ZERO);
  const driveRef = useRef<MoveCommand>(ZERO);
  const tickRef = useRef<number | null>(null);
  const [busy, setBusy] = useState(false);
  const [actionError, setActionError] = useState<string | null>(null);
  const connected = useStatus();

  const submitSpeech = useCallback(async (text: string) => {
    const payload = wrapUserSpeech(text);
    devLog({ transcript: text, payload });
    setActionError(null);
    try {
      await dimos.submitQuery(payload);
    } catch (err) {
      console.error("submit failed", err);
      setActionError("Couldn't reach the robot — try again.");
    }
  }, []);

  const { recording, transcript, error, start, stop } = useStt(submitSpeech);
  useEffect(() => {
    recordingRef.current = recording;
  }, [recording]);

  // Pressing the talk button is a user gesture: unlock iOS speech and stop any
  // current playback so it doesn't talk over the new command.
  const handleStart = useCallback(() => {
    setActionError(null); // clear any stale "couldn't reach the robot" notice
    cancelSpeech(); // stop any current playback first…
    unlockSpeech(); // …then prime within this gesture
    start();
  }, [start]);

  const toggleTts = useCallback(() => {
    // side effects outside the state updater (StrictMode double-invokes updaters)
    const next = !ttsEnabledRef.current;
    setTtsEnabled(next);
    if (next) {
      unlockSpeech();
      speak("Speech on"); // audible confirmation, spoken inside this tap (gesture)
    } else {
      cancelSpeech();
    }
  }, []);

  async function handleAction(a: QuickAction) {
    setBusy(true);
    setActionError(null);
    try {
      // Route through the agent (so it narrates "I'm standing up…"), same path
      // as a voice command — but no <user_speech> wrap since this is a button.
      await dimos.submitQuery(a.command);
    } catch (err) {
      console.error("command failed", err);
      setActionError("Couldn't reach the robot — try again.");
    } finally {
      setBusy(false);
    }
  }

  async function handleInterrupt() {
    markIdle();
    cancelSpeech();
    try {
      await dimos.interrupt();
    } catch (err) {
      console.error("interrupt failed", err);
    }
  }

  // Joystick: reflect every frame in the readout, emit move_command over the
  // teleop socket, re-emit at ~15Hz while held, zero twist on release.
  const handleMove = useCallback(
    (m: MoveCommand) => {
      driveRef.current = m;
      setDrive(m);
      if (tickRef.current === null) {
        teleop.drive(m);
        tickRef.current = window.setInterval(
          () => teleop.drive(driveRef.current),
          66,
        );
      }
    },
    [teleop.drive],
  );

  const handleDriveEnd = useCallback(() => {
    if (tickRef.current !== null) {
      clearInterval(tickRef.current);
      tickRef.current = null;
    }
    driveRef.current = ZERO;
    setDrive(ZERO);
    teleop.stop();
  }, [teleop.stop]);

  useEffect(
    () => () => {
      if (tickRef.current !== null) clearInterval(tickRef.current);
    },
    [],
  );

  return (
    <>
      <Head>
        <title>Goldie</title>
      </Head>
      <main className="mx-auto flex min-h-screen w-full max-w-[440px] flex-col">
        <Header
          connected={connected}
          ttsEnabled={ttsEnabled}
          onToggleTts={toggleTts}
        />
        <ModeToggle mode={mode} onChange={setMode} />
        <div className="flex flex-1 flex-col px-5 pb-6 pt-3">
          {mode === "voice" ? (
            <VoicePanel
              messages={messages}
              active={active}
              recording={recording}
              transcript={transcript || undefined}
              error={error || actionError || undefined}
              busy={busy}
              onStart={handleStart}
              onStop={stop}
              onAction={handleAction}
              onInterrupt={handleInterrupt}
            />
          ) : (
            <ManualPanel
              drive={drive}
              busy={busy}
              linkConnected={teleop.connected}
              linkConfigured={teleop.configured}
              onMove={handleMove}
              onEnd={handleDriveEnd}
              onAction={handleAction}
            />
          )}
        </div>
      </main>
    </>
  );
}
