import type { EpochMs } from "../../protocol/time.ts";
import type { TimelineRow } from "./timeline.ts";
import type { ToolCall } from "@earendil-works/pi-ai";
import type { ToolResult } from "../../protocol/conversation/state.ts";
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
