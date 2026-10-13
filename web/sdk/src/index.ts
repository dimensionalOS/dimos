// Public root of @dimos/sdk. React-free by design: the React bindings live
// in the ./react subpath so a non-React consumer never pulls React in.

export {
  connect,
  type ConnectOptions,
  EMPTY_MANIFEST,
  type PublishOptions,
  type PublishReceipt,
  type Session,
} from "./session.ts";
export { ChannelStore, StatusStore } from "./store.ts";
export type { ChannelSnapshot, ChannelStats, SessionStatus, Slot } from "./store.ts";
export { createDecoderRegistry, DecoderRegistry } from "./decoders/index.ts";
export type { Decoded, Decoder } from "./decoders/index.ts";
export { type CostmapValue, inflateCostmap } from "./decoders/costmap.ts";
export { inflateVoxels, type VoxelsValue } from "./decoders/voxels.ts";
export {
  isLcmSchema,
  lcmDecoder,
  type LcmDecoderOptions,
  type LcmSchema,
  type LcmValue,
  MAX_LCM_ARRAY_ELEMENTS,
} from "./decoders/lcm.ts";
export {
  PublishError,
  type PublishOutcome,
  type SessionError,
  type SessionErrorCode,
  WatchRejectedError,
} from "./errors.ts";
export type { RelayInfo, TransportDeps, TransportPhase } from "./transport.ts";
export type { PeerConnectionFactory, PeerConnectionLike } from "./rtc.ts";
export type {
  ChannelSpec,
  FrameHeader,
  IceServer,
  JsonValue,
  PanelSpec,
  RobotInfo,
} from "@dimos/shared";
export { TRACK_ENCODING } from "@dimos/shared/manifest";
export type { Manifest } from "@dimos/shared/manifest";
