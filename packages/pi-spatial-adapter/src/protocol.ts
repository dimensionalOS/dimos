import { Type, type Static } from "typebox";

export const PROTOCOL_VERSION = 2;
export const MAX_LINE_BYTES = 64 * 1024;
export const MAX_PENDING_REQUESTS = 4;
export const MAX_PROMPT_BYTES = 32 * 1024;

const BudgetSchema = Type.Object({
  maxTurns: Type.Integer({ minimum: 1, maximum: 100 }),
  maxToolCalls: Type.Integer({ minimum: 1, maximum: 100 }),
  timeoutMs: Type.Integer({ minimum: 1_000, maximum: 900_000 }),
}, { additionalProperties: false });
const RunConfigSchema = Type.Object({
  promptMode: Type.Union([Type.Literal("visualization_forbidden"), Type.Literal("visualization_encouraged")]),
  answerType: Type.Union([Type.Literal("boolean"), Type.Literal("integer")]),
  modelId: Type.Literal("gpt-5.6-luna"),
  thinkingLevel: Type.Literal("medium"),
  implementationDigests: Type.Object({
    adapter: Type.String({ pattern: "^[^@]+@sha256:[0-9a-f]{64}$" }),
    scorer: Type.String({ pattern: "^[^@]+@sha256:[0-9a-f]{64}$" }),
    protocol: Type.String({ pattern: "^[^@]+@sha256:[0-9a-f]{64}$" }),
  }, { additionalProperties: false }),
}, { additionalProperties: false });

export const RunStartSchema = Type.Object({
  version: Type.Literal(PROTOCOL_VERSION), type: Type.Literal("run_start"), id: Type.String({ minLength: 1, maxLength: 128 }),
  prompt: Type.String({ minLength: 1, maxLength: MAX_PROMPT_BYTES }), budget: BudgetSchema, config: RunConfigSchema,
}, { additionalProperties: false });
export const ToolReplySchema = Type.Object({
  version: Type.Literal(PROTOCOL_VERSION), type: Type.Literal("tool_reply"), id: Type.String({ minLength: 1, maxLength: 128 }),
  ok: Type.Boolean(), result: Type.Optional(Type.Unknown()), error: Type.Optional(Type.String({ maxLength: 1024 })),
}, { additionalProperties: false });
export type RunStart = Static<typeof RunStartSchema>;
export type ToolReply = Static<typeof ToolReplySchema>;
export type InboundFrame = RunStart | ToolReply;

export type SessionEvidenceFrame = {
  state: "complete" | "partial" | "unavailable";
  persisted: boolean;
  relative_path?: string;
  system_prompt?: {
    relative_path: "pi-prompt/system.txt";
    byte_count: number;
    sha256: string;
  };
  initial_prompt?: {
    relative_path: "pi-prompt/initial.txt";
    byte_count: number;
    sha256: string;
  };
};

export type OutboundFrame =
  | { version: 2; type: "run_started"; id: string; tools: readonly string[] }
  | { version: 2; type: "tool_call"; id: string; tool: string; params: Record<string, unknown> }
  | { version: 2; type: "transcript"; event: string; delta?: string }
  | { version: 2; type: "run_complete"; id: string; ok: boolean; reason: "submitted" | "max_turns" | "max_tool_calls" | "timeout" | "session_error" | "protocol_error" | "pre_image_policy_violation" | "post_image_policy_violation"; session_evidence: SessionEvidenceFrame }
  | { version: 2; type: "protocol_error"; error: string };

function record(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}
function exactKeys(value: Record<string, unknown>, keys: readonly string[]): boolean {
  return Object.keys(value).every((key) => keys.includes(key)) && keys.every((key) => key in value || key === "result" || key === "error");
}
function integer(value: unknown, min: number, max: number): value is number {
  return typeof value === "number" && Number.isInteger(value) && value >= min && value <= max;
}

export function parseFrame(line: string): InboundFrame {
  if (Buffer.byteLength(line, "utf8") > MAX_LINE_BYTES) throw new Error("NDJSON frame exceeds limit");
  let value: unknown;
  try { value = JSON.parse(line); } catch { throw new Error("invalid JSON frame"); }
  if (!record(value) || value.version !== PROTOCOL_VERSION || typeof value.type !== "string") throw new Error("invalid protocol v2 frame");
  if (value.type === "run_start") {
    const budget = value.budget;
    const config = value.config;
    if (!exactKeys(value, ["version", "type", "id", "prompt", "budget", "config"]) || typeof value.id !== "string" ||
        value.id.length === 0 || value.id.length > 128 || typeof value.prompt !== "string" || value.prompt.length === 0 ||
        Buffer.byteLength(value.prompt, "utf8") > MAX_PROMPT_BYTES || !record(budget) || !exactKeys(budget, ["maxTurns", "maxToolCalls", "timeoutMs"]) ||
        !integer(budget.maxTurns, 1, 100) || !integer(budget.maxToolCalls, 1, 100) || !integer(budget.timeoutMs, 1_000, 900_000) ||
        !record(config) || !exactKeys(config, ["promptMode", "answerType", "modelId", "thinkingLevel", "implementationDigests"]) ||
        (config.promptMode !== "visualization_forbidden" && config.promptMode !== "visualization_encouraged") ||
        (config.answerType !== "boolean" && config.answerType !== "integer") || config.modelId !== "gpt-5.6-luna" ||
        config.thinkingLevel !== "medium" || !validImplementationDigests(config.implementationDigests)) throw new Error("invalid run_start frame");
    return value as RunStart;
  }
  if (value.type === "tool_reply" && exactKeys(value, ["version", "type", "id", "ok", "result", "error"]) &&
      typeof value.id === "string" && value.id.length > 0 && value.id.length <= 128 && typeof value.ok === "boolean" &&
      (value.error === undefined || (typeof value.error === "string" && value.error.length <= 1024))) return value as ToolReply;
  throw new Error("invalid or unsupported protocol v2 frame");
}

export function encodeFrame(frame: OutboundFrame): string {
  const line = JSON.stringify(frame);
  if (Buffer.byteLength(line, "utf8") > MAX_LINE_BYTES) throw new Error("outbound NDJSON frame exceeds limit");
  return `${line}\n`;
}

function validImplementationDigests(value: unknown): boolean {
  if (!record(value) || !exactKeys(value, ["adapter", "scorer", "protocol"])) return false;
  return ["adapter", "scorer", "protocol"].every((key) => typeof value[key] === "string" && /^[^@]+@sha256:[0-9a-f]{64}$/.test(value[key] as string));
}
