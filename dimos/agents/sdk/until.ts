import type { Live } from "#dimos/agents/shared/protocol/status.ts";

/** Resolves with the first non-undefined `pick` of the readable's values, now or later; print mode waits for its outcome so. */
export function until<T, R>(readable: Live<T>, pick: (value: T) => R | undefined, signal?: AbortSignal): Promise<R> {
  return new Promise<R>((resolve, reject) => {
    const now = pick(readable.value);
    if (now !== undefined) {
      resolve(now);
      return;
    }
    if (signal?.aborted) {
      reject(signal.reason);
      return;
    }
    const stop = readable.subscribe(() => {
      const next = pick(readable.value);
      if (next === undefined) return;
      finish();
      resolve(next);
    });
    const aborted = (): void => {
      finish();
      reject(signal?.reason);
    };
    signal?.addEventListener("abort", aborted, { once: true });
    function finish(): void {
      stop();
      signal?.removeEventListener("abort", aborted);
    }
  });
}
