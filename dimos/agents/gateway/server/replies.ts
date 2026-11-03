import type { Context } from "@earendil-works/chord/context";
import type { Outcome } from "#dimos/agents/shared/protocol/outcome.ts";

type Reply = { readonly outcome: Promise<Outcome<unknown>>; readonly at: number };

/**
 * The first outcome of each `by_request_id` call, kept for `keepMs`, across every object of the gateway process: a view
 * object can be disposed and made again between a dropped call and its retry, so the cache cannot live in one. A call
 * still running is joined, not started twice; a call that threw is forgotten, so a retry runs it again. A call runs in
 * `context`, which only gateway shutdown aborts: the link of the client that started it can close before it settles.
 */
export class Replies {
  private readonly entries = new Map<string, Reply>();
  private readonly keepMs: number;
  private readonly context: Context;

  constructor(keepMs: number, context: Context) {
    this.keepMs = keepMs;
    this.context = context;
  }

  once<T>(key: string, run: (context: Context) => Promise<Outcome<T>>): Promise<Outcome<T>> {
    const now = Date.now();
    for (const [known, reply] of this.entries) if (now - reply.at > this.keepMs) this.entries.delete(known);
    const kept = this.entries.get(key);
    // One key names one action's arguments, so the outcome stored under it has this call's value type.
    if (kept !== undefined) return kept.outcome as Promise<Outcome<T>>;
    const outcome = run(this.context);
    this.entries.set(key, { outcome, at: now });
    outcome.catch(() => {
      if (this.entries.get(key)?.outcome === outcome) this.entries.delete(key);
    });
    return outcome;
  }
}
