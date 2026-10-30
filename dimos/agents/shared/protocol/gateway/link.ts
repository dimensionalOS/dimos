import type { EpochMs } from "../time.ts";

/** The gateway's one connection to McpServer, as terminals and the event intake see it. */
export type LinkStatus =
  | { readonly state: "connecting"; readonly attempt: number; readonly lastError: string | null }
  | { readonly state: "up"; readonly since: EpochMs; readonly server: string }
  | { readonly state: "down"; readonly error: string; readonly retryAt: EpochMs };

/**
 * The link in one clause, for the model, the log and every frontend: its state and what last went wrong.
 */
export function describeLink(status: LinkStatus): string {
  switch (status.state) {
    case "connecting":
      return status.lastError === null
        ? `the link is connecting (attempt ${status.attempt})`
        : `the link is reconnecting (attempt ${status.attempt}, after: ${status.lastError})`;
    case "up":
      return `the link is up (${status.server})`;
    case "down":
      return `the link is down (${status.error}; next try in ${Math.max(0, Math.ceil((status.retryAt - Date.now()) / 1000))} s)`;
  }
}
