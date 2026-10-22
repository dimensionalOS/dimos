// The map3d panel's worker: inflates a voxels.zlib.v1 frame, unpacks it into
// voxel centres and paints the height colours, then hands both buffers back
// transferred. At a million voxels that is ~50 ms the page's main thread no
// longer spends per frame; voxelPrepare.ts drives it.
import { inflateVoxels, type VoxelsValue } from "@dimos/sdk";
import { heightColors } from "./voxelView.ts";

export interface VoxelRequest {
  id: number;
  value: VoxelsValue;
}

export interface VoxelReply {
  id: number;
  positions?: Float32Array;
  colors?: Uint8Array;
  error?: string;
}

// The worker scope, typed by hand: the DOM lib's `self` is a Window.
const scope = self as unknown as {
  onmessage: ((e: MessageEvent<VoxelRequest>) => void) | null;
  postMessage(message: VoxelReply, transfer: Transferable[]): void;
};

scope.onmessage = async ({ data }) => {
  try {
    const positions = await inflateVoxels(data.value);
    const colors = heightColors(positions);
    scope.postMessage({ id: data.id, positions, colors }, [
      positions.buffer as ArrayBuffer,
      colors.buffer as ArrayBuffer,
    ]);
  } catch (e) {
    scope.postMessage({ id: data.id, error: e instanceof Error ? e.message : String(e) }, []);
  }
};
