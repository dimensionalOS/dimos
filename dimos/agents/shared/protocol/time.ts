/** Milliseconds since the Unix epoch. */
export type EpochMs = number & { readonly __brand: "EpochMs" };

export function epochMs(ms: number): EpochMs {
  return ms as EpochMs;
}
