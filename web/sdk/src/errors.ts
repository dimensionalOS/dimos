// Structured session-level errors surfaced through SessionStatus.lastError,
// and the rejection type for watch() promises that can no longer resolve.

export type SessionErrorCode = "invalid_manifest" | "unknown_channel" | "relay_error";

export interface SessionError {
  code: SessionErrorCode;
  /** Human-readable form; the cockpit status bar renders it verbatim. */
  message: string;
  /** The offending channel, set for unknown_channel. */
  ch?: string;
}

export class WatchRejectedError extends Error {
  readonly reason: "closed" | "superseded";

  constructor(reason: "closed" | "superseded") {
    super(`watch ${reason}`);
    this.name = "WatchRejectedError";
    this.reason = reason;
  }
}
