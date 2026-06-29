// Widget registry (Rung 6): pick a visualization component from the message
// TYPE. This is what makes the tool a generic "framework" rather than a
// hard-coded demo — adding support for a new message type is one line here.
import type { ComponentType } from "react";
import { JsonInspector } from "./JsonInspector";
import { PoseReadout } from "./PoseReadout";

type PanelProps = { topic: string };

const BY_TYPE: Record<string, ComponentType<PanelProps>> = {
  "geometry_msgs.PoseStamped": PoseReadout,
  // "sensor_msgs.LaserScan": ScanStats,   // ← extend like this
  // "nav_msgs.OccupancyGrid": GridStats,
};

/** Choose a widget for a message type, falling back to the JSON inspector. */
export function widgetForType(type: string): ComponentType<PanelProps> {
  return BY_TYPE[type] ?? JsonInspector;
}
