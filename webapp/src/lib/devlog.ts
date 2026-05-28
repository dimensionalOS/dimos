/**
 * Fire-and-forget log to the Next dev server terminal (same-origin /api/log),
 * independent of the DimOS backend. See src/pages/api/log.ts.
 */
export function devLog(entry: Record<string, unknown>): void {
  fetch("/api/log", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(entry),
  }).catch(() => {
    // logging must never break the app
  });
}
