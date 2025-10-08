import * as pako from 'pako';

export interface EncodedOptimizedGrid {
  update_type: "full" | "delta";
  shape: [number, number];
  dtype: string;
  compressed: boolean;
  compression?: "zlib" | "none";
  data?: string;
  chunks?: Array<{
    pos: [number, number];
    size: [number, number];
    data: string;
  }>;
}

export class OptimizedGrid {
  private fullGrid: Uint8Array | null = null;
  private shape: [number, number] = [0, 0];

  decode(msg: EncodedOptimizedGrid): Float32Array {
    if (msg.update_type === "full") {
      return this.decodeFull(msg);
    } else {
      return this.decodeDelta(msg);
    }
  }

  private decodeFull(msg: EncodedOptimizedGrid): Float32Array {
    if (!msg.data) {
      throw new Error("Missing data for full update");
    }

    const binaryString = atob(msg.data);
    const compressed = new Uint8Array(binaryString.length);
    for (let i = 0; i < binaryString.length; i++) {
      compressed[i] = binaryString.charCodeAt(i);
    }

    // Decompress if needed
    let decompressed: Uint8Array;
    if (msg.compressed && msg.compression === "zlib") {
      decompressed = pako.inflate(compressed);
    } else {
      decompressed = compressed;
    }

    // Store for delta updates
    this.fullGrid = decompressed;
    this.shape = msg.shape;

    // Convert uint8 back to float32 costmap values
    const float32Data = new Float32Array(decompressed.length);
    for (let i = 0; i < decompressed.length; i++) {
      // Map 255 back to -1 for unknown cells
      const val = decompressed[i]!;
      float32Data[i] = val === 255 ? -1 : val;
    }

    return float32Data;
  }

  private decodeDelta(msg: EncodedOptimizedGrid): Float32Array {
    if (!this.fullGrid) {
      console.warn("No full grid available for delta update - skipping until full update arrives");
      const size = msg.shape[0] * msg.shape[1];
      return new Float32Array(size).fill(-1);
    }

    if (!msg.chunks) {
      throw new Error("Missing chunks for delta update");
    }

    // Apply delta updates to the full grid
    for (const chunk of msg.chunks) {
      const [y, x] = chunk.pos;
      const [h, w] = chunk.size;

      // Decode chunk data
      const binaryString = atob(chunk.data);
      const compressed = new Uint8Array(binaryString.length);
      for (let i = 0; i < binaryString.length; i++) {
        compressed[i] = binaryString.charCodeAt(i);
      }

      let decompressed: Uint8Array;
      if (msg.compressed && msg.compression === "zlib") {
        decompressed = pako.inflate(compressed);
      } else {
        decompressed = compressed;
      }

      // Update the full grid with chunk data
      const width = this.shape[1];
      let chunkIdx = 0;
      for (let cy = 0; cy < h; cy++) {
        for (let cx = 0; cx < w; cx++) {
          const gridIdx = (y + cy) * width + (x + cx);
          const val = decompressed[chunkIdx++];
          if (val !== undefined) {
            this.fullGrid[gridIdx] = val;
          }
        }
      }
    }

    // Convert to float32
    const float32Data = new Float32Array(this.fullGrid.length);
    for (let i = 0; i < this.fullGrid.length; i++) {
      const val = this.fullGrid[i]!;
      float32Data[i] = val === 255 ? -1 : val;
    }

    return float32Data;
  }

  getShape(): [number, number] {
    return this.shape;
  }
}
