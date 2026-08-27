import type { FrameHeader } from "@dimos/shared";
import type { Decoded } from "./index.ts";

// fatal: corrupted bytes must fail decode, not U+FFFD their way onto screen.
const utf8 = new TextDecoder("utf-8", { fatal: true });

// Parsing and previewing run on the main thread: a payload above this cap is
// reported instead of parsed, so a huge but valid frame cannot freeze the tab
// (the outer MAX_DATA_FRAME_BYTES is 64 MiB). Reference scale: the Python
// viewer caps pose.json.v1 at 64 KiB.
export const MAX_JSON_PAYLOAD_BYTES = 256 * 1024;

// Longest preview the raw-value UI ever mounts in the DOM.
export const JSON_PREVIEW_MAX_CHARS = 2048;

/** Decoder for every `*.json.vN` encoding (pose.json.v1 and friends). */
export function jsonDecoder(payload: Uint8Array, _header: FrameHeader): Decoded {
  if (payload.byteLength > MAX_JSON_PAYLOAD_BYTES) {
    return {
      value: undefined,
      preview: `(oversized json payload: ${payload.byteLength} B, cap ${MAX_JSON_PAYLOAD_BYTES} B)`,
    };
  }
  const text = utf8.decode(payload);
  const value: unknown = JSON.parse(text);
  const preview = text.length > JSON_PREVIEW_MAX_CHARS
    ? `${text.slice(0, JSON_PREVIEW_MAX_CHARS)} ... (truncated, ${text.length} chars)`
    : text;
  return { value, preview };
}
