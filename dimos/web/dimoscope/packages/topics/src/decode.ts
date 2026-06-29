// Decoding helpers — the only place that touches @dimos/msgs.
// The 8-byte hash at the head of every payload self-describes the type, so a
// single decode() works for any transport (gateway LCM packets or Zenoh samples).
import { decode as msgsDecode } from "@dimos/msgs";

/** Split a DimOS channel "<topic>#<pkg>.<Type>" into its parts. */
export function splitChannel(channel: string): { topic: string; type: string } {
  const h = channel.indexOf("#");
  return h >= 0
    ? { topic: channel.slice(0, h), type: channel.slice(h + 1) }
    : { topic: channel, type: "?" };
}

/** Decode a message body (bytes starting with the 8-byte type hash). */
export function decodeBody(payload: Uint8Array): unknown {
  return msgsDecode(payload);
}

/** Best-effort source timestamp (ms) from a std_msgs/Header on the message. */
export function srcTsMs(data: unknown): number | undefined {
  const stamp = (data as any)?.header?.stamp ?? (data as any)?.header?.ts ?? (data as any)?.ts;
  if (typeof stamp === "number") {
    // Heuristic: ns (>1e15) | ms (>1e12) | s (else).
    if (stamp > 1e15) return stamp / 1e6;
    if (stamp > 1e12) return stamp;
    return stamp * 1000;
  }
  if (stamp && typeof stamp === "object") {
    const sec = (stamp as any).sec ?? (stamp as any).secs ?? 0;
    const nsec = (stamp as any).nsec ?? (stamp as any).nanosec ?? (stamp as any).nanos ?? 0;
    return sec * 1000 + nsec / 1e6;
  }
  return undefined;
}
