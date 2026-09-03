// Plain English for the SDK's tx refusal codes. The codes (web/sdk/src
// /session.ts) name internals - "not_tx", "bad_channel" - that mean nothing
// to someone driving a robot, and every panel that sends needs the same
// wording, so the map lives here rather than in one of them.

const TX_REASON: Record<string, string> = {
  disconnected: "not connected",
  no_manifest: "not connected",
  unknown_channel: "channel missing from manifest",
  not_tx: "channel is not writable",
  too_large: "message too long",
  bad_channel: "bad channel",
};

/** The reason in plain English; an unrecognised code passes through so a
 * newer relay's reason is still shown rather than swallowed. */
export function txReasonText(reason: string): string {
  return TX_REASON[reason] ?? reason;
}
