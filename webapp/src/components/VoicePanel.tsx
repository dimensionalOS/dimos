import type { AgentMessage } from "@/lib/agentMessage";
import type { QuickAction } from "@/lib/types";
import VoiceButton from "./VoiceButton";
import StatusCard from "./StatusCard";
import QuickActions from "./QuickActions";
import InterruptButton from "./InterruptButton";

export default function VoicePanel({
  messages,
  active,
  recording,
  transcript,
  error,
  busy,
  onStart,
  onStop,
  onAction,
  onInterrupt,
}: {
  messages: AgentMessage[];
  active: boolean;
  recording: boolean;
  transcript?: string;
  error?: string;
  busy?: boolean;
  onStart: () => void;
  onStop: () => void;
  onAction: (a: QuickAction) => void;
  onInterrupt: () => void;
}) {
  return (
    <div className="flex flex-1 flex-col">
      <VoiceButton
        recording={recording}
        transcript={transcript}
        error={error}
        onStart={onStart}
        onStop={onStop}
      />
      <StatusCard messages={messages} active={active} />
      {active ? (
        <InterruptButton onInterrupt={onInterrupt} />
      ) : (
        <QuickActions onAction={onAction} disabled={busy} />
      )}
    </div>
  );
}
