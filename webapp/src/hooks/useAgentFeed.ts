import { useEffect, useRef, useState } from "react";
import * as dimos from "@/lib/dimos";
import type { StreamStatus } from "@/lib/dimos";
import { classifyAgentMessage, type AgentMessage } from "@/lib/agentMessage";

/**
 * Subscribe to the `agent_responses` stream and expose a clean, de-duplicated
 * feed of the last few messages (warnings/keepalives filtered out), an `active`
 * flag while the agent is replying, and the connection status. `onMessage`
 * fires once per new message (used to speak it aloud).
 */
export function useAgentFeed(opts?: {
  key?: string;
  limit?: number;
  onMessage?: (m: AgentMessage) => void;
}) {
  const key = opts?.key ?? "agent_responses";
  const limit = opts?.limit ?? 5;
  const onMessageRef = useRef(opts?.onMessage);
  onMessageRef.current = opts?.onMessage;

  const [messages, setMessages] = useState<AgentMessage[]>([]);
  const [status, setStatus] = useState<StreamStatus>("connecting");
  const [active, setActive] = useState(false);
  const lastTextRef = useRef<string | null>(null);
  const idleTimer = useRef<number | null>(null);

  useEffect(() => {
    const unsubscribe = dimos.subscribeStream(
      key,
      (data) => {
        const msg = classifyAgentMessage(data);
        if (!msg || msg.kind === "warning") return; // drop noise
        if (lastTextRef.current === msg.text) return; // drop echoes
        lastTextRef.current = msg.text;

        onMessageRef.current?.(msg); // side effect (speak) — outside the updater
        setMessages((prev) => [...prev, msg].slice(-limit));

        setActive(true);
        if (idleTimer.current) window.clearTimeout(idleTimer.current);
        idleTimer.current = window.setTimeout(() => setActive(false), 5000);
      },
      setStatus,
    );
    return () => {
      unsubscribe();
      if (idleTimer.current) window.clearTimeout(idleTimer.current);
    };
  }, [key, limit]);

  const markIdle = () => {
    setActive(false);
    if (idleTimer.current) window.clearTimeout(idleTimer.current);
  };

  return { messages, active, status, markIdle };
}
