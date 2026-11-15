import type { Live } from "#dimos/agents/shared/protocol/status.ts";

export type Role = "owner" | "viewer";

/** Calls `changed` on every change of any of `members`; returns the one unsubscribe. */
export function listen(members: readonly Live<unknown>[], changed: () => void): () => void {
  const stops = members.map((member) => member.subscribe(changed));
  return () => {
    for (const stop of stops) stop();
  };
}
