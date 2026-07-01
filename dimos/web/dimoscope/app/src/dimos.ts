// The app's typed hooks — bound once to the generated topic map, then imported everywhere instead of
// `@dimos/react`. Every topic-name hook now autocompletes the name + infers the message type with no
// per-call generic:
//
//   import { useTopicLatest } from "../dimos";
//   const { data } = useTopicLatest("/nav/pose");  // data?: geometry_msgs.PoseStamped (autocompletes the name)
//
// dimos.topics.gen.ts is generated STATICALLY from a dimos blueprint (no gateway) — regenerate after the
// blueprint changes:  deno task gen-types scenarios/nav.py --out app/src/dimos.topics.gen.ts
import { createDimosHooks } from "@dimos/react";
import type { DimosCommands, DimosTopics } from "./dimos.topics.gen.ts";

// The typed surface: `useDimosClient()` → DimosClient<DimosTopics, DimosCommands>, topic hooks
// autocomplete the name + infer the message, and `useModules().ScopeNav.navigate_to(goal)` is a typed
// RPC call. Panels import ALL dimos hooks from here (not @dimos/react) so the whole app is typed.
export const {
  useDimosClient,
  useTopicLatest,
  useTopicRef,
  useImageTopic,
  useTopicStats,
  useModules,
} = createDimosHooks<DimosTopics, DimosCommands>();

// Map-agnostic hooks (don't depend on the generated map) re-exported so `../dimos` is the one import
// site for every panel. useRpc/useCommands stay for the gateway's DYNAMIC (author-time-unknown)
// command list; useModules is for known commands.
export {
  useCaps,
  useCommands,
  useRpc,
  useServers,
  useStatus,
  useTeleop,
  useTopics,
  useVideo,
} from "@dimos/react";
