export type AgentMsgKind = "ai" | "tool" | "system";

export interface AgentMessage {
  kind: AgentMsgKind;
  text: string;
}

function normalizeKind(k: unknown): AgentMsgKind {
  return k === "tool" || k === "system" ? k : "ai";
}

function stripWrappingQuotes(t: string): string {
  const m = t.match(/^(['"])([\s\S]*)\1$/);
  return m ? m[2].trim() : t;
}

/**
 * Turn one raw `agent_responses` SSE frame into a classified message, or null
 * for empty/keepalive frames.
 *
 * The backend tags each message as a typed JSON envelope
 * `{ kind: "ai" | "tool" | "system", text }`. Only `ai` messages are the
 * agent's spoken replies; `tool`/`system` are status (shown, never spoken —
 * see index.tsx). Plain-text frames from the legacy backend are treated as `ai`.
 */
export function classifyAgentMessage(raw: string): AgentMessage | null {
  const s = (raw ?? "").trim();
  if (!s) return null;

  // Typed JSON envelope (current backend).
  if (s[0] === "{" || s[0] === "[" || s[0] === '"') {
    try {
      const obj = JSON.parse(s);
      if (obj && typeof obj === "object" && !Array.isArray(obj)) {
        const text = String(obj.text ?? obj.message ?? obj.content ?? "").trim();
        return text ? { kind: normalizeKind(obj.kind), text } : null;
      }
      if (typeof obj === "string") {
        const text = obj.trim();
        return text ? { kind: "ai", text } : null;
      }
    } catch {
      /* not JSON — fall through to legacy plain-text handling */
    }
  }

  // Legacy plain-text frame (pre-envelope backend): treat as a spoken reply.
  return { kind: "ai", text: stripWrappingQuotes(s) };
}
