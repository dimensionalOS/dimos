// URL-persisted settings: read a query param once at load, reflect changes back with replaceState —
// the current configuration stays copy-paste shareable without adding history entries.
export function getParam(name: string): string | null {
  return new URLSearchParams(location.search).get(name);
}

/** Set (or, with null, remove) a query param in place. */
export function setUrlParam(key: string, value: string | null) {
  const u = new URL(location.href);
  if (value === null) u.searchParams.delete(key);
  else u.searchParams.set(key, value);
  history.replaceState(null, "", u);
}
