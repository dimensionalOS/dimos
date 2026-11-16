import { afterEach, describe, expect, it, vi } from "vitest";
import { TopicMediaDecoder } from "./media.ts";
import type { TopicMedia } from "./decoders/topicMedia.ts";
import type { TopicSample } from "./topics.ts";

const sample = (key: string, sequence: number, size = 1): TopicSample => ({
  key,
  sequence,
  payload: new Uint8Array(size),
  metadata: {},
  encoding: "image/jpeg",
  source: "workspace",
  generation: 1,
  ingressTs: 1,
  sentTs: 1,
});
const value: TopicMedia = {
  kind: "image",
  width: 1,
  height: 1,
  encoding: "jpeg",
  data: new Uint8Array(),
  header: { frame_id: "", stamp: 0, seq: 0 },
};
const decoders: TopicMediaDecoder[] = [];
const pending: (() => void)[] = [];
const decode = vi.fn((_sample: TopicSample) =>
  new Promise<TopicMedia>((resolve) => pending.push(() => resolve(value)))
);
function decoder(callback = vi.fn(), error = vi.fn()): TopicMediaDecoder {
  const decoder = new TopicMediaDecoder(callback, error, decode);
  decoders.push(decoder);
  return decoder;
}
async function finish(): Promise<void> {
  const resolve = pending.shift();
  if (!resolve) throw new Error("no active decode");
  resolve();
  await Promise.resolve();
  await Promise.resolve();
  await Promise.resolve();
}
afterEach(async () => {
  for (const decoder of decoders) decoder.close();
  while (pending.length) await finish();
  decoders.length = 0;
  decode.mockClear();
});

describe("latest media scheduling", () => {
  it("runs at most two decodes globally and selects the newest queued sample", async () => {
    const first = decoder(), second = decoder(), seen = vi.fn(), third = decoder(seen);
    first.push(sample("a", 1));
    second.push(sample("b", 1));
    for (let i = 1; i <= 1000; i++) third.push(sample("c", i));
    expect(decode).toHaveBeenCalledTimes(2);
    expect(third.pendingBytes).toBe(1);
    await finish();
    expect(decode.mock.calls[2][0].sequence).toBe(1000);
    await finish();
    await finish();
    expect(seen).toHaveBeenCalledOnce();
    expect(third.pendingBytes).toBe(0);
  });

  it("preserves a quiet wildcard key while a busy key replaces its own slot", async () => {
    const seen = vi.fn(), media = decoder(seen);
    media.push(sample("busy", 1));
    media.push(sample("quiet", 1));
    for (let i = 2; i <= 100; i++) media.push(sample("busy", i));
    await finish();
    await finish();
    await finish();
    expect(seen.mock.calls.map((call) => [call[1].key, call[1].sequence])).toEqual([["busy", 1], [
      "quiet",
      1,
    ], [
      "busy",
      100,
    ]]);
    expect(media.pendingBytes).toBe(0);
  });

  it("never delivers old work after reset or close and retains active bytes until completion", async () => {
    const seen = vi.fn(), media = decoder(seen);
    media.push(sample("a", 1, 100));
    media.reset();
    expect(media.pendingBytes).toBe(100);
    media.push(sample("a", 2));
    await finish();
    media.close();
    await finish();
    expect(seen).not.toHaveBeenCalled();
    expect(media.pendingBytes).toBe(0);
  });

  it("presents completed decodes while a faster producer always has another sample pending", async () => {
    const seen = vi.fn(), media = decoder(seen);
    media.push(sample("camera", 1));
    for (let sequence = 2; sequence <= 5; sequence++) {
      media.push(sample("camera", sequence));
      await finish();
      expect(seen.mock.calls.at(-1)?.[1].sequence).toBe(sequence - 1);
    }
  });

  it("preserves the pending frame when replacements exceed budget and reports overload once", async () => {
    const first = decoder(),
      second = decoder(),
      seen = vi.fn(),
      error = vi.fn(),
      media = decoder(seen, error);
    first.push(sample("a", 1));
    second.push(sample("b", 1));
    media.push(sample("camera", 1));
    for (let sequence = 2; sequence <= 5; sequence++) {
      media.push(sample("camera", sequence, 8 * 1024 * 1024 + 1));
    }
    expect(media.pendingBytes).toBe(1);
    expect(error).toHaveBeenCalledOnce();
    await finish();
    await finish();
    await finish();
    expect(seen.mock.calls.map((call) => call[1].sequence)).toEqual([1]);
  });

  it("reports byte and wildcard-key overflow instead of growing queues", () => {
    const error = vi.fn(), media = decoder(vi.fn(), error);
    media.push(sample("large", 1, 8 * 1024 * 1024));
    media.push(sample("other", 1, 8 * 1024 * 1024));
    media.push(sample("excess", 1));
    expect(error).toHaveBeenCalledOnce();
    expect(media.pendingBytes).toBe(16 * 1024 * 1024);
    media.reset();
    for (let i = 0; i <= 128; i++) media.push(sample(`key/${i}`, 1));
    expect(error).toHaveBeenCalledTimes(2);
  });
});
