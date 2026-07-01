// Typed topic map — a representative sample (a typical Go2 topic set) checked in so the app + typed
// hooks compile. Edit by hand as topics change, or regenerate the type text with `generateTypes()`
// from packages/topics/scripts/genTypes.ts.

import type { geometry_msgs, nav_msgs } from "@dimos/msgs";

export interface DimosTopics {
  "/cmd_vel": geometry_msgs.Twist;
  "/map": nav_msgs.OccupancyGrid;
  "/odom": geometry_msgs.PoseStamped;
}

export type TopicName = keyof DimosTopics;
export type MsgOf<K extends TopicName> = DimosTopics[K];
