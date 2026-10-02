/**
 * Inflate a zlib stream (RFC 1950, what Python's zlib.compress emits) to
 * exactly `expected` bytes. That is DecompressionStream("deflate");
 * "deflate-raw" is RFC 1951 and would reject every frame (the golden vectors
 * in shared/fixtures pin this pairing). Output beyond `expected` throws
 * mid-stream (decompression-bomb guard), a short stream throws at the end,
 * and a corrupt stream rejects from read().
 */
export async function inflateExact(bytes: Uint8Array, expected: number): Promise<Uint8Array> {
  const out = new Uint8Array(expected);
  let written = 0;
  const inflated = new Blob([bytes as BlobPart]).stream()
    .pipeThrough(new DecompressionStream("deflate"));
  const reader = inflated.getReader();
  try {
    while (true) {
      const { done, value: chunk } = await reader.read();
      if (done) break;
      if (written + chunk.length > expected) {
        throw new Error(`stream inflates beyond ${expected} bytes`);
      }
      out.set(chunk, written);
      written += chunk.length;
    }
  } finally {
    void reader.cancel().catch(() => {});
  }
  if (written !== expected) {
    throw new Error(`stream inflated to ${written} bytes, expected ${expected}`);
  }
  return out;
}
