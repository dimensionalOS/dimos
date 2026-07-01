// QoS lanes ≈ ROS 2 named profiles (command·sensor·default·bulk). defaultLane auto-assigns by
// topic/type, resolveQos merges overrides, applyCaps degrades to what a transport honors. Priority
// only matters because the gateway's per-client scheduler enforces it under load (gateway/data.py).
import type { Qos } from "./types.ts";
import type { QosCaps } from "./types.ts";

export type Lane = "command" | "sensor" | "default" | "bulk";

/** Numeric rank for the scheduler — higher wins. Mirrors `Qos.priority` (the server reads this). */
export const PRIORITY_RANK: Record<NonNullable<Qos["priority"]>, number> = {
  critical: 3,
  high: 2,
  normal: 1,
  low: 0,
};

/** The four lanes ≈ ROS 2 profiles: command(Services) · sensor(SensorData) · default(Default) · bulk. */
export const LANES: Record<Lane, Qos> = {
  // control: every message, in order, wins everything (gateway adds clamp + deadman on the WS path).
  command: {
    reliability: "reliable",
    priority: "critical",
    depth: 1,
    conflation: "all",
  },
  // high-rate telemetry: latest-wins, droppable — pose/odom/imu/tf/joints (≈ ROS 2 SensorData).
  sensor: {
    reliability: "best-effort",
    priority: "high",
    depth: 5,
    conflation: "latest",
  },
  // general reliable topics (≈ ROS 2 Default).
  default: {
    reliability: "reliable",
    priority: "normal",
    depth: 10,
    conflation: "all",
  },
  // bulk: lidar/camera/pointcloud/maps — best-effort, lowest priority, yields first under load.
  bulk: {
    reliability: "best-effort",
    priority: "low",
    depth: 1,
    conflation: "latest",
  },
};

// topic-name / message-type keyword → lane. Checked in priority order (command → bulk → sensor → default).
const CMD = /(^|\/)(cmd_vel|cmd|teleop|goal|clicked_point|move_base|nav_goal)(\/|$)/i;
const CMD_TYPE = /(Twist|Goal)/i;
const BULK =
  /(^|\/)(lidar|laser|scan|points?|point_?cloud|cloud|camera|image|depth|rgb|map|costmap|occupancy|grid)(\/|$)/i;
const BULK_TYPE = /(PointCloud|LaserScan|Image|OccupancyGrid|CompressedImage)/i;
const SENSOR = /(^|\/)(pose|odom|imu|tf|joint|battery|twist|velocity|state|wrench)(\/|$)/i;
const SENSOR_TYPE = /(Pose|Odometry|Imu|Transform|JointState|Quaternion|Vector3)/i;

/** Sane default lane for a topic, from its name + message type. Zero-config: every topic lands somewhere. */
export function defaultLane(topic: string, type = ""): Lane {
  if (CMD.test(topic) || CMD_TYPE.test(type)) return "command";
  if (BULK.test(topic) || BULK_TYPE.test(type)) return "bulk"; // big/heavy beats the generic sensor match
  if (SENSOR.test(topic) || SENSOR_TYPE.test(type)) return "sensor";
  return "default";
}

/** Resolve a topic's effective QoS: its default lane's preset, with a per-topic override merged on top.
 *  `override.lane` switches the base preset; remaining fields override individual knobs. */
export function resolveQos(
  topic: string,
  type = "",
  override?: Partial<Qos> & { lane?: Lane },
): Qos {
  const lane = override?.lane ?? defaultLane(topic, type);
  const { lane: _drop, ...fields } = override ?? {};
  return { ...LANES[lane], ...fields };
}

// Always honored by the gateway (client rate-limit + the scheduler inputs), regardless of transport.
const GATEWAY_FIELDS: (keyof Qos)[] = [
  "maxHz",
  "rateLimit",
  "conflation",
  "priority",
  "depth",
];

/** Degrade a QoS to what a transport actually honors: keep the gateway-enforced fields, plus the
 *  gateway-scheduler fields the adapter advertises in `caps.transport`; drop the rest. Called by the
 *  gateway-WS adapter before each subscribe op (gatewayWs.ts) so a client-only transport doesn't send
 *  priority/reliability/depth the server-less path can't honor. */
export function applyCaps(qos: Qos, caps: QosCaps): Qos {
  const honored = new Set<keyof Qos>([...GATEWAY_FIELDS, ...(caps.transport ?? [])]);
  const out: Qos = {};
  for (const k of Object.keys(qos) as (keyof Qos)[]) {
    if (honored.has(k)) (out as Record<string, unknown>)[k] = qos[k];
  }
  return out;
}
