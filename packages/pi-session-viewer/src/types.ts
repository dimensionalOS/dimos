export type SessionStatus = "complete" | "partial";

export interface UsageSummary {
  input?: number;
  output?: number;
  cacheRead?: number;
  cacheWrite?: number;
  reasoning?: number;
  totalTokens?: number;
  cost?: {
    total?: number;
  };
}

export interface TextPart {
  type: "text";
  text: string;
}

export interface ThinkingPart {
  type: "thinking";
  text: string;
  redacted: boolean;
}

export interface ToolCallPart {
  type: "tool_call";
  callId: string;
  name: string;
  arguments: unknown;
}

export interface ToolResultPart {
  type: "tool_result";
  callId: string;
  name: string;
  content: string;
  isError: boolean;
  details?: unknown;
}

export interface ImagePart {
  type: "image";
  mimeType?: string;
  omitted: true;
}

export interface StatePart {
  type: "state";
  label: string;
  value: string;
}

export type SessionPart =
  | TextPart
  | ThinkingPart
  | ToolCallPart
  | ToolResultPart
  | ImagePart
  | StatePart;

export type EntryRole =
  | "user"
  | "assistant"
  | "tool"
  | "custom"
  | "bash"
  | "state";

export interface SessionEntry {
  id: string;
  parentId: string | null;
  depth: number;
  type: string;
  role: EntryRole;
  timestamp: string;
  timestampMs?: number;
  title: string;
  preview: string;
  parts: SessionPart[];
  usage?: UsageSummary;
  model?: string;
  provider?: string;
  stopReason?: string;
  raw: Record<string, unknown>;
}

export interface SessionSummary {
  id: string;
  status: SessionStatus;
  startedAt: string;
  model?: string;
  provider?: string;
  entryCount: number;
  branchCount: number;
  totalUsage: UsageSummary;
}

export interface PiSessionViewModel {
  schemaVersion: "1.0";
  summary: SessionSummary;
  entries: SessionEntry[];
  rootIds: string[];
  headIds: string[];
  defaultHeadId: string;
}
