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

/** "rejected": the publish definitively did not happen. "unknown": it may
 * have (connection loss, timeout - the bridge can publish right before its
 * ack is lost); never assume either way, and never auto-resend. */
export type PublishOutcome = "rejected" | "unknown";

export class PublishError extends Error {
  constructor(
    readonly outcome: PublishOutcome,
    /** Stable machine-readable reason: a local validation code, a relay
     * rejection code, or a bridge nack code. */
    readonly code: string,
    message: string,
  ) {
    super(`${code}: ${message}`);
    this.name = "PublishError";
  }
}
