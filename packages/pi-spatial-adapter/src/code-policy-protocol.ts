export const CODE_POLICY_PROTOCOL_VERSION = 1;
export const CODE_POLICY_MAX_LINE_BYTES = 64 * 1024;

export type CodePolicyInbound =
  | {
      version: 1;
      type: "session_start";
      id: string;
      initial_prompt: string;
      thinking_level: "medium";
    }
  | { version: 1; type: "prompt"; id: string; text: string }
  | {
      version: 1;
      type: "tool_reply";
      id: string;
      ok: boolean;
      result?: string;
      error?: string;
    }
  | { version: 1; type: "abort" }
  | { version: 1; type: "dispose" };

export type CodePolicyOutbound =
  | { version: 1; type: "session_started"; id: string; tools: ["python_exec"] }
  | {
      version: 1;
      type: "tool_call";
      id: string;
      tool: "python_exec";
      params: { code: string; timeout_s?: number };
    }
  | { version: 1; type: "transcript"; event: string }
  | {
      version: 1;
      type: "turn_complete";
      id: string;
      policy_call_count: number;
      final_text: string;
    }
  | {
      version: 1;
      type: "session_closed";
      id: string;
      evidence: {
        state: "complete" | "partial" | "unavailable";
        persisted: boolean;
        relative_path?: string;
        system_prompt?: { relative_path: string; byte_count: number; sha256: string };
        initial_prompt?: { relative_path: string; byte_count: number; sha256: string };
      };
    }
  | { version: 1; type: "protocol_error"; error: string };

function record(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

export function parseCodePolicyFrame(line: string): CodePolicyInbound {
  if (Buffer.byteLength(line, "utf8") > CODE_POLICY_MAX_LINE_BYTES) {
    throw new Error("code-policy frame exceeds limit");
  }
  let value: unknown;
  try {
    value = JSON.parse(line);
  } catch {
    throw new Error("invalid code-policy JSON frame");
  }
  if (!record(value) || value.version !== CODE_POLICY_PROTOCOL_VERSION) {
    throw new Error("invalid code-policy protocol version");
  }
  if (
    value.type === "session_start" &&
    typeof value.id === "string" &&
    value.id.length > 0 &&
    typeof value.initial_prompt === "string" &&
    value.initial_prompt.length > 0 &&
    value.thinking_level === "medium"
  ) {
    return value as CodePolicyInbound;
  }
  if (
    value.type === "prompt" &&
    typeof value.id === "string" &&
    value.id.length > 0 &&
    typeof value.text === "string" &&
    value.text.length > 0
  ) {
    return value as CodePolicyInbound;
  }
  if (
    value.type === "tool_reply" &&
    typeof value.id === "string" &&
    typeof value.ok === "boolean" &&
    (value.result === undefined || typeof value.result === "string") &&
    (value.error === undefined || typeof value.error === "string")
  ) {
    return value as CodePolicyInbound;
  }
  if (value.type === "abort" || value.type === "dispose") {
    return value as CodePolicyInbound;
  }
  throw new Error("invalid code-policy frame");
}

export function encodeCodePolicyFrame(frame: CodePolicyOutbound): string {
  const encoded = JSON.stringify(frame);
  if (Buffer.byteLength(encoded, "utf8") > CODE_POLICY_MAX_LINE_BYTES) {
    throw new Error("outbound code-policy frame exceeds limit");
  }
  return `${encoded}\n`;
}
