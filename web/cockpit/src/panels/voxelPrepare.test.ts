import { describe, expect, it } from "vitest";
import type { VoxelsValue } from "@dimos/sdk";
import { createVoxelPreparer, prepareVoxelsInline, type WorkerLike } from "./voxelPrepare.ts";
import type { VoxelReply, VoxelRequest } from "./voxelWorker.ts";

/** A worker that answers how the test says, recording what it was posted. */
class FakeWorker implements WorkerLike {
  posted: { message: VoxelRequest; transfer: Transferable[] }[] = [];
  onmessage: ((e: MessageEvent<VoxelReply>) => void) | null = null;
  onerror: ((e: ErrorEvent) => void) | null = null;
  postMessage(message: VoxelRequest, transfer: Transferable[]): void {
    this.posted.push({ message, transfer });
  }
  reply(data: VoxelReply): void {
    this.onmessage?.({ data } as MessageEvent<VoxelReply>);
  }
  fail(message: string): void {
    this.onerror?.({ message } as ErrorEvent);
  }
}

const value: VoxelsValue = { bytes: new Uint8Array([1, 2, 3]), res: 0.05, n: 1, chunks: 1 };

describe("createVoxelPreparer", () => {
  it("posts a transferred copy of the frame and routes replies by id", async () => {
    const workers: FakeWorker[] = [];
    const prepare = createVoxelPreparer(() => {
      const w = new FakeWorker();
      workers.push(w);
      return w;
    });
    const a = prepare(value);
    const b = prepare(value);
    expect(workers.length).toBe(1); // one worker, created on the first request
    const [first, second] = workers[0].posted;
    expect(first.message.id).not.toBe(second.message.id);
    expect(first.message.value.bytes).toEqual(value.bytes);
    expect(first.message.value.bytes.buffer).not.toBe(value.bytes.buffer); // the store keeps its frame
    expect(first.transfer).toEqual([first.message.value.bytes.buffer]);

    const positions = new Float32Array([1, 2, 3]);
    workers[0].reply({ id: second.message.id, positions, colors: new Uint8Array([9, 9, 9]) });
    expect((await b).positions).toBe(positions);
    workers[0].reply({ id: first.message.id, error: "corrupt zlib" });
    await expect(a).rejects.toThrow("corrupt zlib");
    workers[0].reply({ id: 999, positions, colors: new Uint8Array(3) }); // unknown id: ignored
  });

  it("a worker failure rejects what is pending and the next request gets a fresh worker", async () => {
    const workers: FakeWorker[] = [];
    const prepare = createVoxelPreparer(() => {
      const w = new FakeWorker();
      workers.push(w);
      return w;
    });
    const pending = prepare(value);
    workers[0].fail("boom");
    await expect(pending).rejects.toThrow("voxel worker failed: boom");
    void prepare(value);
    expect(workers.length).toBe(2);
  });
});

describe("prepareVoxelsInline", () => {
  it("rejects a frame whose payload is not a zlib stream", async () => {
    await expect(prepareVoxelsInline(value)).rejects.toThrow();
  });
});
