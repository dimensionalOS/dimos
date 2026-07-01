// Channel + header parsing helpers for the client's hot path.

/** Split a DimOS channel "<topic>#<pkg>.<Type>" into its parts. */
export function splitChannel(channel: string): { topic: string; type: string } {
  const h = channel.indexOf("#");
  return h >= 0
    ? { topic: channel.slice(0, h), type: channel.slice(h + 1) }
    : { topic: channel, type: "?" };
}

/** Best-effort source timestamp (ms) from a std_msgs/Header — the *fallback* latency input used
 *  only when the gateway didn't stamp the WS hop (`gatewaySendMs`); see client.ts onSample. The
 *  heuristics exist because there's no single canonical stamp field/epoch across message types
 *  (ns vs ms vs s, `stamp` vs `ts`, `{sec,nsec}` variants). */
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

/**
 * Best-effort per-topic sequence number, for wire drop/gap detection. The bench
 * source stamps `frame_id = str(seq)`, so a purely-numeric frame_id is read as a
 * counter; real frames ("base_link", "odom") are not numeric and yield undefined.
 * Falls back to a `header.seq` field if the message carries one.
 */
export function seqFrom(data: unknown): number | undefined {
  const fid = (data as any)?.frame_id ?? (data as any)?.header?.frame_id;
  if (typeof fid === "string" && fid.length > 0 && fid.length < 16 && /^\d+$/.test(fid)) {
    return Number(fid);
  }
  const seq = (data as any)?.header?.seq;
  return typeof seq === "number" ? seq : undefined;
}
