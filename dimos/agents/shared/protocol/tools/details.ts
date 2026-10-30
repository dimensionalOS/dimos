import type { Static } from "typebox";
import type { TaskId } from "@earendil-works/pi-durable";
import { LimitRecordSchema } from "./limit.ts";
import { RenderDetailsSchema } from "../media.ts";
import { Type } from "typebox";

/** pi-durable's `TruncationResult` without its content: the record Pi's read and bash cards read. */
const TruncationSchema = Type.Object({
  truncated: Type.Boolean(),
  truncatedBy: Type.Union([Type.Literal("lines"), Type.Literal("bytes"), Type.Null()]),
  totalLines: Type.Integer({ minimum: 0 }),
  totalBytes: Type.Integer({ minimum: 0 }),
  outputLines: Type.Integer({ minimum: 0 }),
  outputBytes: Type.Integer({ minimum: 0 }),
  lastLinePartial: Type.Boolean(),
  firstLineExceedsLimit: Type.Boolean(),
  maxLines: Type.Integer({ minimum: 0 }),
  maxBytes: Type.Integer({ minimum: 0 }),
});

/** Set by the afterTool hook on every result: what the `max_chars` cut removed. */
const limit = Type.Union([LimitRecordSchema, Type.Null()]);

const render = Type.Union([RenderDetailsSchema, Type.Null()]);

/** Set on the result a call returns when its work moves to the background: the task that finishes the work. */
const BackgroundSchema = Type.Object({ task: Type.Unsafe<TaskId>(Type.Integer({ minimum: 1 })) });

const background = Type.Optional(BackgroundSchema);

/**
 * What a call whose work moved to the background returns as its raw details, and as its structured output when the
 * tool declares a schema for one.
 */
export const MovedSchema = Type.Object({ background: BackgroundSchema });

/** The `details` each tool of the plan's table stores on its results. Every type of stored details is derived from here. */
export const DETAILS = {
  read: Type.Object({ truncation: Type.Optional(TruncationSchema), limit }),
  bash: Type.Object({
    /** pi-durable's bash keeps neither of these; its settle step builds them from `structuredOutput` and the spill file for Pi's card. Null when nothing was cut. */
    truncation: Type.Union([TruncationSchema, Type.Null()]),
    fullOutputPath: Type.Union([Type.String(), Type.Null()]),
    /** Null when the command did not exit on its own, such as a timeout. */
    exitCode: Type.Union([Type.Integer(), Type.Null()]),
    /** Files the command created or named, rendered. */
    render,
    background,
    limit,
  }),
  edit: Type.Object({
    diff: Type.String(),
    patch: Type.String(),
    firstChangedLine: Type.Optional(Type.Integer({ minimum: 1 })),
    limit,
  }),
  write: Type.Object({ limit }),
  inventory: Type.Object({ render, background, limit }),
  /** `render` is the exported file a `stream_get export=true` result points to. */
  stream_get: Type.Object({ render, background, limit }),
  agent_module: Type.Object({ render, background, limit }),
  job: Type.Object({ limit }),
};

export type DetailsByTool = { readonly [N in keyof typeof DETAILS]: Static<(typeof DETAILS)[N]> };

export type Background = Static<typeof BackgroundSchema>;

/** The agent's tools, exactly the plan's table. */
export type AgentToolName = keyof DetailsByTool;

/** The HarnessRuntime skills offered as tools; their schemas come from McpServer. */
export type RuntimeToolName = Extract<AgentToolName, "inventory" | "stream_get" | "agent_module">;

/** A result's `details` as read back from the store: the tool's own shape, keyed by the tool that wrote it. */
export type ToolDetails =
  | { readonly [N in AgentToolName]: { readonly tool: N; readonly details: DetailsByTool[N] } }[AgentToolName]
  /** A tool outside the table, from an older transcript, or details this build's schema rejects. */
  | { readonly tool: null };
