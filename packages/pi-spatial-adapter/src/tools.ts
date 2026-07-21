import { readFileSync } from "node:fs";
import { Type, type Static, type TSchema } from "typebox";
import type { ToolDefinition } from "@earendil-works/pi-coding-agent";
import type { OutboundFrame, ToolReply } from "./protocol.js";

export const TOOL_NAMES = ["sandbox_exec", "read_generated_image", "submit_answer"] as const;
export type ToolName = (typeof TOOL_NAMES)[number];
export type AnswerType = "boolean" | "integer";
export type PromptMode = "visualization_forbidden" | "visualization_encouraged";
export const SECOND_SANDBOX_CUE = "Pre-image sandbox budget is exhausted. Your next and only permitted tool is read_generated_image using the relative PNG path you created. Do not call sandbox_exec or submit_answer.";
export const IMAGE_RETRY_CUE = "Image read failed. You have one final image-read attempt. Call read_generated_image once with the relative path of an existing valid PNG under /work. Do not call sandbox_exec or submit_answer.";
export const FINAL_IMAGE_CUE = "Use your final image-read attempt now with read_generated_image. Do not call sandbox_exec or submit_answer.";
type ToolSchemaArtifact = {
  schema_version: "1.0";
  tools: ToolDefinition[];
  submit_answer: ToolDefinition & { variants: Record<AnswerType, TSchema> };
};
const TOOL_SCHEMA_ARTIFACT = JSON.parse(
  readFileSync(new URL("./tool-definitions.v1.json", import.meta.url), "utf8"),
) as ToolSchemaArtifact;
export const ToolInventorySchema = Type.Array(Type.Union([
  Type.Literal("sandbox_exec"),
  Type.Literal("read_generated_image"),
  Type.Literal("submit_answer"),
]), { minItems: 3, maxItems: 3, uniqueItems: true });
export type ToolInventory = Static<typeof ToolInventorySchema>;

export function assertToolInventory(names: readonly string[]): asserts names is readonly ToolName[] {
  if (names.length !== TOOL_NAMES.length || names.some((name) => !TOOL_NAMES.includes(name as ToolName)) ||
      new Set(names).size !== TOOL_NAMES.length) throw new Error("tool inventory is not exactly the approved allowlist");
}

export function customTools(tools: readonly ToolDefinition[]): ToolDefinition[] {
  const names = tools.map((tool) => tool.name);
  assertToolInventory(names);
  return [...tools];
}

export function assertNoBuiltinTools(names: readonly string[]): void {
  if (names.some((name) => !TOOL_NAMES.includes(name as ToolName))) throw new Error(`unexpected tool: ${names.find((name) => !TOOL_NAMES.includes(name as ToolName))}`);
}

export const SandboxExecParameters = Type.Object({ command: Type.String({ minLength: 1, maxLength: 4096 }) }, { additionalProperties: false });
export const ReadGeneratedImageParameters = Type.Object({
  path: Type.String({ minLength: 1, maxLength: 512, pattern: "^(?!/)(?!.*(?:^|/)\\.\\.(?:/|$))[A-Za-z0-9._/-]+$" }),
}, { additionalProperties: false });
export function submitAnswerParameters(answerType: AnswerType) {
  return TOOL_SCHEMA_ARTIFACT.submit_answer.variants[answerType];
}

export interface ToolBroker {
  request(tool: ToolName, params: Record<string, unknown>): Promise<ToolReplyResult>;
  reply(frame: ToolReply): void;
  close(reason: string): void;
  toolCallCount(): number;
  sandboxAttempts(): number;
  submissionAccepted(): boolean;
  visualizationSatisfied(): boolean;
}
export type ToolReplyResult = { text: string } | { image: { mimeType: "image/png"; data: string } };

export function createBroker(send: (frame: OutboundFrame) => void, maxPending: number, maxCalls = 100, answerType: AnswerType = "boolean"): ToolBroker {
  let sequence = 0;
  let acceptedSubmission = false;
  let visualizationSatisfied = false;
  let sandboxAttemptCount = 0;
  let imageRecovery = false;
  let finalImageRequired = false;
  const pending = new Map<string, { tool: ToolName; resolve: (result: ToolReplyResult) => void; reject: (error: Error) => void }>();
  const reply = (frame: ToolReply): void => {
    const entry = pending.get(frame.id);
    if (!entry) throw new Error("unknown or duplicate tool reply");
    pending.delete(frame.id);
    if (!frame.ok) {
      if (frame.error === "image_read_failed_retry_once" && typeof frame.result === "object" && frame.result !== null && !Array.isArray(frame.result)) {
        imageRecovery = true;
        entry.resolve({ text: JSON.stringify(frame.result) });
      } else entry.reject(new Error(frame.error ?? "host tool failed"));
      return;
    }
    try { entry.resolve(validateToolResult(frame.result, entry.tool)); } catch (error) { entry.reject(error instanceof Error ? error : new Error("invalid tool result")); }
  };
  const request = (tool: ToolName, params: Record<string, unknown>): Promise<ToolReplyResult> => {
    if (pending.size >= maxPending) return Promise.reject(new Error("too many outstanding tool requests"));
    if (sequence >= maxCalls) return Promise.reject(new Error("tool-call budget exceeded"));
    if (tool === "read_generated_image" && !validWorkspaceImagePath(params.path)) return Promise.reject(new Error("image path must be relative to /work"));
    if (tool === "submit_answer" && !validAnswer(params.answer, answerType)) return Promise.reject(new Error("answer does not match the public answer type"));
    const id = `tool-${++sequence}`;
    return new Promise<ToolReplyResult>((resolve, reject) => {
      pending.set(id, { tool, resolve, reject });
      send({ version: 2, type: "tool_call", id, tool, params });
    }).then((result) => {
      if (tool === "submit_answer" && "text" in result) {
        acceptedSubmission = isAcceptedSubmitReceipt(result.text, answerType);
      }
      if (tool === "read_generated_image" && "image" in result) visualizationSatisfied = true;
      if (tool === "sandbox_exec") {
        sandboxAttemptCount += 1;
        if (imageRecovery) finalImageRequired = true;
      }
      return result;
    });
  };
  const close = (reason: string): void => {
    for (const entry of pending.values()) entry.reject(new Error(reason));
    pending.clear();
  };
  return {
    request,
    reply,
    close,
    toolCallCount: () => sequence,
    sandboxAttempts: () => sandboxAttemptCount,
    submissionAccepted: () => acceptedSubmission,
    visualizationSatisfied: () => visualizationSatisfied,
  };
}

function isAcceptedSubmitReceipt(text: string, answerType: AnswerType): boolean {
  let value: unknown;
  try { value = JSON.parse(text); } catch { return false; }
  if (typeof value !== "object" || value === null || Array.isArray(value)) return false;
  const receipt = value as Record<string, unknown>;
  return Object.keys(receipt).length === 3 &&
    typeof receipt.accepted === "boolean" && receipt.accepted === true &&
    typeof receipt.instance_id === "string" && receipt.instance_id.length > 0 &&
    receipt.answer_type === answerType;
}

function validateToolResult(value: unknown, tool: ToolName): ToolReplyResult {
  if (typeof value === "string" && value.length <= 16_384 && tool !== "read_generated_image") return { text: value };
  if (typeof value === "object" && value !== null && !Array.isArray(value)) {
    const result = value as Record<string, unknown>;
    if (typeof result.text === "string" && result.text.length <= 16_384) return { text: result.text };
    if (result.mime === "image/png" && typeof result.data === "string" && /^[A-Za-z0-9+/]*={0,2}$/.test(result.data) && result.data.length <= 5_600_000) {
      const bytes = Buffer.from(result.data, "base64");
      if (bytes.length <= 4 * 1024 * 1024 && bytes.subarray(0, 8).equals(Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]))) return { image: { mimeType: "image/png", data: result.data } };
    }
  }
  throw new Error("invalid host tool result");
}

export function toolDefinitions(
  broker: ToolBroker,
  answerType: AnswerType,
  promptMode: PromptMode = "visualization_forbidden",
): ToolDefinition[] {
  const make = (name: ToolName, label: string, description: string, parameters: ToolDefinition["parameters"]): ToolDefinition => ({
    name, label, description, parameters,
    execute: async (_id, params) => {
      let result: ToolReplyResult;
      try {
        result = await broker.request(name, params as Record<string, unknown>);
      } catch (error) {
        if (name === "read_generated_image" && error instanceof Error && error.message === "image_read_failed_retry_once") {
          return { content: [{ type: "text", text: IMAGE_RETRY_CUE }], details: {} };
        }
        throw error;
      }
      if ("image" in result) {
        const content: Array<
          | { type: "image"; data: string; mimeType: "image/png" }
          | { type: "text"; text: string }
        > = [{ type: "image", data: result.image.data, mimeType: result.image.mimeType }];
        if (name === "read_generated_image" && promptMode === "visualization_encouraged") {
          content.push({
            type: "text",
            text: "Visualization requirement satisfied. Stop further analysis now. Call submit_answer exactly once with your best answer from the evidence already available, even if uncertain. Do not call sandbox_exec or read_generated_image again.",
          });
        }
        return { content, details: {} };
      }
      const recovery = name === "read_generated_image" ? imageRecoveryContract(result.text) : undefined;
      const content = recovery
        ? [{ type: "text" as const, text: JSON.stringify(recovery) }, { type: "text" as const, text: IMAGE_RETRY_CUE }]
        : [{ type: "text" as const, text: result.text }];
      if (name === "sandbox_exec" && promptMode === "visualization_encouraged" && broker.sandboxAttempts() === 3) {
        content.push({ type: "text" as const, text: FINAL_IMAGE_CUE });
      } else if (name === "sandbox_exec" && promptMode === "visualization_encouraged" && broker.sandboxAttempts() === 2) {
        content.push({ type: "text" as const, text: SECOND_SANDBOX_CUE });
      }
      return { content, details: {} };
    },
  });
  return [
    ...TOOL_SCHEMA_ARTIFACT.tools.map((tool) =>
      make(tool.name as ToolName, tool.label ?? tool.name, tool.description ?? "", tool.parameters),
    ),
    make(
      TOOL_SCHEMA_ARTIFACT.submit_answer.name as ToolName,
      TOOL_SCHEMA_ARTIFACT.submit_answer.label ?? TOOL_SCHEMA_ARTIFACT.submit_answer.name,
      TOOL_SCHEMA_ARTIFACT.submit_answer.description ?? "",
      submitAnswerParameters(answerType),
    ),
  ];
}

function imageRecoveryContract(text: string): Record<string, unknown> | undefined {
  try {
    const value: unknown = JSON.parse(text);
    if (typeof value !== "object" || value === null || Array.isArray(value)) return undefined;
    const record = value as Record<string, unknown>;
    if (typeof record.category !== "string" || typeof record.sandbox_attempts !== "number" || typeof record.image_attempts !== "number") return undefined;
    return { category: record.category, sandbox_attempts: record.sandbox_attempts, image_attempts: record.image_attempts };
  } catch { return undefined; }
}

function validAnswer(value: unknown, answerType: AnswerType): boolean {
  return answerType === "boolean"
    ? typeof value === "boolean"
    : typeof value === "number" && Number.isInteger(value) && value >= 0 && value <= 2_147_483_647;
}

function validWorkspaceImagePath(value: unknown): value is string {
  return typeof value === "string" && value.length > 0 && value.length <= 512 && /^(?!\/)(?!.*(?:^|\/)\.\.(?:\/|$))[A-Za-z0-9._/-]+$/.test(value);
}
