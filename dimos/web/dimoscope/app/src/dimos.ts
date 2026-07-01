// The app's typed hooks — bound once to the generated topic map, then imported everywhere instead of
// `@dimos/react`. Every topic-name hook now autocompletes the name + infers the message type with no
// per-call generic:
//
//   import { useTopicLatest } from "../dimos";
//   const { data } = useTopicLatest("/odom");   // data?: geometry_msgs.PoseStamped  (autocompletes "/odom")
//
// Regenerate the map after topics change:  deno task dtop gen-types --out app/src/dimos.topics.gen.ts
import { createDimosHooks } from "@dimos/react";
import type { DimosTopics } from "./dimos.topics.gen.ts";

export const { useTopicLatest, useTopicRef, useImageTopic, useTopicStats } =
  createDimosHooks<DimosTopics>();
