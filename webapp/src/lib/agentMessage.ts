export type AgentMsgKind = "spoke" | "status" | "warning" | "message";

export interface AgentMessage {
  kind: AgentMsgKind;
  text: string;
}

const COMPLETION = /(command executed successfully|goal reached|navigation complete)/i;

/**
 * Turn one raw `agent_responses` frame (plain text, sometimes JSON) into a
 * classified message, or null for empty/keepalive frames.
 */
export function classifyAgentMessage(raw: string): AgentMessage | null {
  let t = (raw ?? "").trim();
  if (!t) return null;

  // some frames may arrive as JSON — pull out the human text
  if (t[0] === "{" || t[0] === "[" || t[0] === '"') {
    try {
      const obj = JSON.parse(t);
      if (typeof obj === "string") t = obj;
      else if (obj && typeof obj === "object")
        t = String(obj.text ?? obj.message ?? obj.content ?? JSON.stringify(obj));
    } catch {
      // not JSON — leave as-is
    }
    t = t.trim();
  }

  // strip quotes only when they wrap the entire string
  const wrapped = t.match(/^(['"])([\s\S]*)\1$/);
  if (wrapped) t = wrapped[2].trim();
  if (!t) return null;

  if (/^warning\b:?\s*/i.test(t))
    return { kind: "warning", text: t.replace(/^warning\b:?\s*/i, "").trim() };
  if (/^spoke\b:?\s*/i.test(t))
    return { kind: "spoke", text: t.replace(/^spoke\b:?\s*/i, "").trim() };
  if (COMPLETION.test(t)) return { kind: "status", text: t };
  return { kind: "message", text: t };
}
