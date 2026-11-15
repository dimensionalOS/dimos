/**
 * A held reference's state members as an object's own: the same readables, no copy. A reference cannot be spread, so
 * the keys come from the shared schema record.
 */
export function forward<R, const K extends keyof R & string>(ref: R, keys: readonly K[]): Pick<R, K> {
  const picked: Partial<Pick<R, K>> = {};
  for (const key of keys) picked[key] = ref[key];
  return picked as Pick<R, K>;
}
