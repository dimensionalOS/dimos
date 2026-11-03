import type { CallId, MovedCall } from "#dimos/agents/shared/protocol/conversation/calls.ts";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { TimelineRow } from "#dimos/agents/shared/view-models/conversation/timeline.ts";
import type { ToolCall } from "@earendil-works/pi-ai";
import type { ToolResult } from "#dimos/agents/shared/protocol/conversation/state.ts";
import type { ToolSlot } from "@earendil-works/pi-durable";

/** Requirement 5 outside the chat (switcher, web page): the running calls and how long each has run. */
export type RunningCall = {
  readonly call: ToolCall;
  readonly slot: ToolSlot;
  readonly backgroundable: boolean;
  readonly forMs: number | null;
};

/** The tool rows of a timeline, with durations from each detachable call's start. */
export function runningCalls(rows: readonly TimelineRow[], now: EpochMs): readonly RunningCall[] {
  const calls: RunningCall[] = [];
  for (const row of rows) {
    if (row.kind !== "tool") continue;
    calls.push({
      call: row.call,
      slot: row.slot,
      backgroundable: row.backgroundable,
      forMs: row.since === null ? null : now - row.since,
    });
  }
  return calls;
}

/** The call Ctrl+B sends to the background: the newest backgroundable running call, or null. */
export function backgroundTarget(calls: readonly RunningCall[]): RunningCall | null {
  for (let index = calls.length - 1; index >= 0; index -= 1) {
    const call = calls[index];
    if (call.backgroundable) return call;
  }
  return null;
}

/** Whether the call's work moved to a background task: its result came back at once and the work reports later. */
export function movedToBackground(result: ToolResult): boolean {
  const details = result.details;
  return details.tool !== null && "background" in details.details && details.details.background !== undefined;
}

/** A call running in the background as a list shows it. */
export type MovedRow = {
  readonly call: CallId;
  readonly tool: string;
  /** `$ command` for bash, the tool and its arguments otherwise. */
  readonly label: string;
  readonly forMs: number;
  /** The newest line it wrote; empty before any output. */
  readonly lastLine: string;
};

export function movedRows(moved: readonly MovedCall[], now: EpochMs): readonly MovedRow[] {
  return moved.map((call) => {
    const command = call.arguments["command"];
    return {
      call: call.call,
      tool: call.tool,
      label:
        call.tool === "bash" && typeof command === "string"
          ? `$ ${command}`
          : `${call.tool} ${JSON.stringify(call.arguments)}`,
      forMs: now - call.since,
      lastLine:
        call.output
          .split("\n")
          .findLast((line) => line.trim() !== "")
          ?.trim() ?? "",
    };
  });
}
