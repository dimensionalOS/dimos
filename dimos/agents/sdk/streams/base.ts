import type { Channel, RateSample } from "#dimos/agents/shared/protocol/streams.ts";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";

/**
 * A decoded SDK preview for the terminal painter. `at` is when the client received the message.
 */
export type FrameRecord =
  | { readonly kind: "rates"; readonly sample: RateSample }
  | {
      readonly kind: "image";
      readonly channel: Channel;
      readonly at: EpochMs;
      readonly width: number;
      readonly height: number;
      readonly jpeg: Uint8Array;
    }
  | { readonly kind: "cloud"; readonly channel: Channel; readonly at: EpochMs; readonly xyz: Float32Array }
  | {
      readonly kind: "grid";
      readonly channel: Channel;
      readonly at: EpochMs;
      readonly width: number;
      readonly height: number;
      /** Metres per cell. */
      readonly resolution: number;
      /** World x, y and yaw of cell 0's corner. */
      readonly origin: readonly [number, number, number];
      /** Row-major from the origin: -1 unknown, 0 free to 100 occupied. */
      readonly cells: Int8Array;
    }
  /** At most 200 characters of a message that is not a picture. */
  | { readonly kind: "value"; readonly channel: Channel; readonly at: EpochMs; readonly text: string }
  /** A wanted channel the source cannot decode. */
  | { readonly kind: "error"; readonly channel: Channel; readonly text: string };

/** Requested preview frames and SDK received rates and source ages. */
export interface FrameSource {
  /** The whole wanted set; a channel left out stops getting frames. */
  want(channels: ReadonlySet<Channel>): void;
  onRecord(listener: (record: FrameRecord) => void): () => void;
  close(): void;
}
