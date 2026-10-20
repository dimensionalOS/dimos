import { channel, decodeCdr, encodeCdr, sensorPacket } from "./cdr.ts";
import { ServerPhysics } from "./physics.ts";
import { ServerLidar } from "./lidar.ts";
import { DimosBridge } from "../../src/bridge.ts";
import { decodePacket, encodeSmallMessage } from "../vendor/lcm/transport.ts";
import type { LCM } from "../vendor/lcm/lcm.ts";
import RAPIER from "@dimforge/rapier3d-compat";

function assert(value: unknown, message: string): asserts value {
  if (!value) throw new Error(message);
}
function hex(data: Uint8Array): string {
  return Array.from(data, (value) => value.toString(16).padStart(2, "0")).join(
    "",
  );
}
function bytes(value: string): Uint8Array {
  return Uint8Array.from(
    value.match(/../g) ?? [],
    (part) => parseInt(part, 16),
  );
}
function rejects(operation: () => unknown): void {
  let failed = false;
  try {
    operation();
  } catch {
    failed = true;
  }
  assert(failed, "malformed payload must fail without a legacy fallback");
}
const input = JSON.parse(
  await Deno.readTextFile(Deno.env.get("DIMOS_DIMSIM_CDR_FIXTURES")!),
);
const output: Record<string, string> = {};

Deno.test("Python little/big-endian fixtures retain all fields through canonical CDR", () => {
  for (const fixture of input.values) {
    const value = decodeCdr(fixture.type, bytes(fixture.hex));
    assert(
      hex(encodeCdr(fixture.type, value)) === fixture.canonicalHex,
      fixture.type,
    );
  }
  const kind = "geometry_msgs/msg/Twist";
  rejects(() => decodeCdr(kind, new Uint8Array([0, 2, 0, 0])));
  rejects(() => decodeCdr(kind, new Uint8Array([1, 0, 0, 0])));
  rejects(() => decodeCdr(kind, new Uint8Array([0, 1, 0])));
  rejects(() => decodeCdr(kind, bytes(input.command + "00")));
  rejects(() => encodeCdr("missing/msg/Type", {}));
});

Deno.test("actual browser methods publish raw RGBA and clamped millimeter depth", () => {
  const previousLocation = Object.getOwnPropertyDescriptor(
    globalThis,
    "location",
  );
  const previousClock = Date.now;
  Object.defineProperty(globalThis, "location", {
    configurable: true,
    value: { protocol: "http:", host: "fixture", search: "" },
  });
  Date.now = () => 1_700_000_000_001;
  try {
    const packets: Uint8Array[] = [];
    const socket = {
      readyState: WebSocket.OPEN,
      send: (data: Uint8Array) => packets.push(data),
    } as unknown as WebSocket;
    const bridge = new DimosBridge({
      agent: {},
      sensorSources: {
        captureRgb: () => ({
          width: 2,
          height: 1,
          data: new Uint8Array([1, 2, 3, 255, 4, 5, 6, 128]),
        }),
        captureDepth: () => ({
          width: 3,
          height: 2,
          data: new Float32Array([-1, 0, 0.5, 1.25, 100, NaN]),
        }),
      },
    });
    bridge.wsRgb = socket;
    bridge.wsDepth = socket;
    bridge._publishImages();
    assert(packets.length === 2, "both sensors must publish");
    for (const packet of packets) {
      const decoded = decodePacket(packet);
      assert(decoded?.type === "small", "WS retains LC02 envelope");
      const key = decoded.channel.startsWith("color_image/")
        ? "color"
        : "depth";
      output[key] = hex(decoded.data);
    }
    const large = sensorPacket(
      channel("/color_image", "sensor_msgs/msg/Image"),
      {
        header: { stamp: { sec: 1, nanosec: 1 }, frame_id: "camera" },
        width: 640,
        height: 288,
        encoding: "rgba8",
        is_bigendian: 0,
        step: 2560,
        data: new Uint8Array(640 * 288 * 4),
      },
    );
    assert(
      large.length > 65535 && decodePacket(large)?.type === "small",
      "WS envelope must not impose UDP limit",
    );
  } finally {
    Date.now = previousClock;
    if (previousLocation) {
      Object.defineProperty(globalThis, "location", previousLocation);
    } else Reflect.deleteProperty(globalThis, "location");
  }
});

Deno.test("real bounded Rapier command, deadman and lidar emit CDR through raw LCM boundary", async () => {
  await RAPIER.init();
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  world.createCollider(
    RAPIER.ColliderDesc.cuboid(10, 0.1, 10).setTranslation(0, -0.1, 0),
  );
  const receivers: ((packet: Uint8Array) => void)[] = [];
  const poses: Uint8Array[] = [];
  const previousClock = Date.now;
  let clock = 1_700_000_000_001;
  Date.now = () => clock;
  let poseDone!: () => void;
  let lidarDone!: () => void;
  const poseReady = new Promise<void>((resolve) => {
    poseDone = resolve;
  });
  const lidarReady = new Promise<void>((resolve) => {
    lidarDone = resolve;
  });
  const network = {
    subscribePacket: (receiver: (packet: Uint8Array) => void) => {
      receivers.push(receiver);
      return () => {};
    },
    getNextSeq: () => 0,
    publishRaw: (name: string, data: Uint8Array) => {
      if (name === channel("/odom", "geometry_msgs/msg/PoseStamped")) {
        poses.push(data);
        // Advance the source clock past the unchanged deadman deadline, without sleeping.
        if (poses.length === 1) clock += 501;
        if (poses.length === 2) poseDone();
      } else if (name === channel("/lidar", "sensor_msgs/msg/PointCloud2")) {
        output.cloud = hex(data);
        lidarDone();
      }
      return Promise.resolve();
    },
  } as unknown as LCM;
  const physics = new ServerPhysics(network, world, RAPIER, new Set());
  const lidar = new ServerLidar(network, world, RAPIER, new Set());
  let timer: ReturnType<typeof setTimeout> | undefined;
  try {
    const deadline = new Promise<never>((_, reject) => {
      timer = setTimeout(
        () => reject(new Error("bounded physics/lidar did not publish")),
        5000,
      );
    });
    physics.start();
    const command = encodeSmallMessage(
      channel("/cmd_vel", "geometry_msgs/msg/Twist"),
      bytes(input.command),
      1,
    );
    for (const receive of receivers) receive(command);
    await Promise.race([poseReady, deadline]);
    physics.stop();
    output.pose = hex(poses[0]);
    output.stoppedPose = hex(poses[1]);
    lidar.setExcludeBody(physics.getBody());
    lidar.updatePose(0, 1, 0, 0, 0, 0, 1);
    lidar.start();
    await Promise.race([lidarReady, deadline]);
  } finally {
    if (timer !== undefined) clearTimeout(timer);
    physics.stop();
    lidar.stop();
    world.free();
    Date.now = previousClock;
  }
  await Deno.writeTextFile(
    Deno.env.get("DIMOS_DIMSIM_CDR_RESULT")!,
    JSON.stringify(output),
  );
});
