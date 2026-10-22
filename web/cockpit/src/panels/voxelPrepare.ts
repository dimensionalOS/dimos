// Turning a voxels.zlib.v1 frame into what the scene draws (voxel centres
// and height colours) in a Worker, so a big map costs the main thread a
// message instead of tens of milliseconds. One worker serves every map3d
// panel of the page; where workers do not exist the same work runs inline.
import { inflateVoxels, type VoxelsValue } from "@dimos/sdk";
import { heightColors } from "./voxelView.ts";
import type { VoxelReply, VoxelRequest } from "./voxelWorker.ts";

export interface PreparedVoxels {
  positions: Float32Array;
  colors: Uint8Array;
}

export type PrepareVoxels = (value: VoxelsValue) => Promise<PreparedVoxels>;

/** The worker surface a preparer drives: a Worker, or a fake in tests. */
export interface WorkerLike {
  postMessage(message: VoxelRequest, transfer: Transferable[]): void;
  onmessage: ((e: MessageEvent<VoxelReply>) => void) | null;
  onerror: ((e: ErrorEvent) => void) | null;
}

/**
 * A preparer over one worker, created on the first request. Requests are
 * numbered and replies routed by id; a worker failure rejects everything
 * pending and the next request starts a fresh worker.
 */
export function createVoxelPreparer(makeWorker: () => WorkerLike): PrepareVoxels {
  let worker: WorkerLike | null = null;
  let nextId = 1;
  const pending = new Map<
    number,
    { resolve: (p: PreparedVoxels) => void; reject: (e: Error) => void }
  >();
  const ensure = (): WorkerLike => {
    if (worker !== null) return worker;
    const started = makeWorker();
    started.onmessage = ({ data }) => {
      const request = pending.get(data.id);
      if (request === undefined) return;
      pending.delete(data.id);
      if (data.error !== undefined || data.positions === undefined || data.colors === undefined) {
        request.reject(new Error(data.error ?? "voxel worker returned nothing"));
      } else {
        request.resolve({ positions: data.positions, colors: data.colors });
      }
    };
    started.onerror = (e) => {
      for (const request of pending.values()) {
        request.reject(new Error(`voxel worker failed: ${e.message}`));
      }
      pending.clear();
      worker = null;
    };
    worker = started;
    return started;
  };
  return (value) =>
    new Promise((resolve, reject) => {
      const id = nextId++;
      pending.set(id, { resolve, reject });
      // The store keeps the frame; the worker takes over a copy.
      const bytes = value.bytes.slice();
      ensure().postMessage({ id, value: { ...value, bytes } }, [bytes.buffer as ArrayBuffer]);
    });
}

/** The same work on the calling thread. */
export async function prepareVoxelsInline(value: VoxelsValue): Promise<PreparedVoxels> {
  const positions = await inflateVoxels(value);
  return { positions, colors: heightColors(positions) };
}

let shared: PrepareVoxels | null = null;

/** The page's preparer: one worker for every map3d panel, or inline where
 * there are no workers. */
export function prepareVoxels(value: VoxelsValue): Promise<PreparedVoxels> {
  if (shared === null) {
    shared = typeof Worker === "function"
      ? createVoxelPreparer(() =>
        new Worker(new URL("./voxelWorker.ts", import.meta.url), {
          type: "module",
        }) as unknown as WorkerLike
      )
      : prepareVoxelsInline;
  }
  return shared(value);
}
