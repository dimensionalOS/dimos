import type { AgentToolName, DetailsByTool } from "#dimos/agents/shared/protocol/tools/details.ts";
import type { Context } from "@earendil-works/chord/context";
import type { ToolExecutionResult, ToolHookCall, ToolRegistration } from "@earendil-works/pi-durable";

/** A result with the tool's own details; the afterTool hook adds `limit` and stores it. */
export type Settled<N extends AgentToolName> = Omit<ToolExecutionResult, "details"> & {
  readonly details: Omit<DetailsByTool[N], "limit">;
};

/**
 * Turns a tool's raw result into what the transcript keeps, before the `max_chars` cut. It runs inside the
 * afterTool hook, which may run again after a crash, so it must give the same result for the same input.
 */
type Settle<N extends AgentToolName> = (
  result: ToolExecutionResult,
  call: ToolHookCall,
  context: Context,
) => Promise<Settled<N>>;

/** A file that already holds a result's whole text, so the `max_chars` cut points there instead of writing a copy. */
type FullTextAt = (result: ToolExecutionResult, call: ToolHookCall) => string | null;

export type ToolPolicy<N extends AgentToolName> = {
  /** Only tools that change nothing are "safe" and run again after a crash. */
  readonly replay: "safe" | "unsafe";
  readonly settle: Settle<N> | null;
  readonly fullTextAt: FullTextAt | null;
  /** Calls whose result is not cut by the default: `max_chars` and the context clamp still apply. */
  readonly wholeResult: ((call: ToolHookCall) => boolean) | null;
  /** Null when a running call of this tool cannot be sent to the background. */
  readonly background: {
    /** The details of the result a moved call returns at once; no settle step runs on it, its work is not done. */
    readonly details: Omit<DetailsByTool[N], "limit" | "background">;
  } | null;
};

/** Every tool the agent has implements this. Only `agentTool()` builds one. */
export type AgentTool<N extends AgentToolName> = ToolPolicy<N> & {
  readonly name: N;
  /** The pi-durable registration: parameters with `max_chars` added, `execute` that strips it, `replay` set. */
  readonly registration: ToolRegistration;
};
