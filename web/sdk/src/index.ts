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
export {
  PublishError,
  type PublishOutcome,
  type SessionError,
  type SessionErrorCode,
  WatchRejectedError,
} from "./errors.ts";
export type { RelayInfo, TransportDeps, TransportPhase } from "./transport.ts";
export type { ChannelSpec, FrameHeader, JsonValue, PanelSpec, RobotInfo } from "@dimos/shared";
export type { Manifest } from "@dimos/shared/manifest";
