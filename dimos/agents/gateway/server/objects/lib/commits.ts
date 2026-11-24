import type { CommitPublication, CommonDocDefinition, Harness, JsonObject } from "@earendil-works/pi-durable";

/** Keeps commits arriving during cell initialization until Chord starts the object's listener. */
export function commitEvents(harness: Harness) {
  const pending: CommitPublication[] = [];
  let listener: ((publication: CommitPublication) => void) | null = null;
  const stop = harness.subscribeCommits((publication) => {
    if (listener === null) pending.push(publication);
    else listener(publication);
  });
  return {
    pending,
    subscribe(next: (publication: CommitPublication) => void): () => void {
      listener = next;
      for (const publication of pending.splice(0)) next(publication);
      // A reload subscribes the new listener before it unsubscribes the old one.
      return () => {
        if (listener === next) listener = null;
      };
    },
    close(): void {
      stop();
      listener = null;
      pending.length = 0;
    },
  };
}

/** A committed document's value as its token types it. */
export function valueOf<T extends JsonObject>(
  value: JsonObject,
  _token: { readonly definition: CommonDocDefinition<T> },
): T {
  return value as T;
}
