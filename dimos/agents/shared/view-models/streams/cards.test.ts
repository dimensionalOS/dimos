import assert from "node:assert/strict";
import type { Channel, MsgType, StreamRef } from "#dimos/agents/shared/protocol/streams.ts";
import type { ModuleInstance } from "#dimos/agents/shared/protocol/runtime/module.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { ratesOf, rateLevel } from "#dimos/agents/shared/view-models/streams/rates.ts";
import { systemGroups } from "#dimos/agents/shared/view-models/streams/cards.ts";
import { test } from "node:test";

const NOW = epochMs(1_760_700_000_000);

function ref(module: string, name: string, type: string, direction: "out" | "inout"): StreamRef {
  return {
    source: { module: module as ModuleInstance, name, direction, type: type as MsgType },
    encoding:
      new Map([
        ["Image", "jpeg.v1"],
        ["PointCloud2", "voxels.zlib.v1"],
        ["OccupancyGrid", "costmap.zlib.v1"],
      ]).get(type) ?? "json.v1",
    channel: `${module}.${name}` as Channel,
  };
}

const GO2 = [
  ref("GO2Connection", "pointcloud", "PointCloud2", "out"),
  ref("GO2Connection", "color_image", "Image", "out"),
  ref("GO2Connection", "odom", "PoseStamped", "out"),
  ref("GO2Connection", "lidar", "PointCloud2", "out"),
  ref("VoxelGridMapper", "global_map", "PointCloud2", "out"),
  ref("CostMapper", "global_costmap", "OccupancyGrid", "out"),
];

test("the robot's picture streams all show under their publisher", () => {
  const { groups, others } = systemGroups(GO2, [], new Set(), new Map(), NOW);
  assert.deepEqual(
    groups.map((group) => [group.title, group.cards.map((card) => `${card.title}:${card.picture ?? "rate"}`)]),
    [
      ["GO2Connection", ["pointcloud:cloud", "color_image:image", "lidar:cloud"]],
      ["VoxelGridMapper", ["global_map:cloud"]],
      ["CostMapper", ["global_costmap:grid"]],
    ],
  );
  assert.equal(others, 1);
});

test("a channel the conversation already shows is left out", () => {
  const shown = new Set([`GO2Connection.color_image` as Channel]);
  const { groups } = systemGroups(GO2, [], shown, new Map(), NOW);
  assert.deepEqual(
    groups.flatMap((group) => group.cards.map((card) => `${card.title}:${card.picture ?? "rate"}`)),
    ["pointcloud:cloud", "lidar:cloud", "global_map:cloud", "global_costmap:grid"],
  );
});

test("cards use the SDK received rate and age without a local sample window", () => {
  const camera = GO2[1];
  const rates = ratesOf({ at: NOW, counts: [{ channel: camera.channel, hz: 12.5, ageS: 4 }] });
  const { groups } = systemGroups([camera], [], new Set(), rates, NOW);
  const card = groups[0]?.cards[0];
  assert.equal(card?.rate?.hz, 12.5);
  assert.equal(card?.rate?.lastAt, NOW - 4_000);
  assert.equal(card?.level, "warn");
  assert.equal(rateLevel(rates.get(camera.channel), epochMs(NOW + 26_000)), "critical");
  const fresh = ratesOf({ at: NOW, counts: [{ channel: camera.channel, hz: 30, ageS: 0.01 }] });
  assert.equal(fresh.get(camera.channel)?.hz, 30);
  assert.equal(rateLevel(fresh.get(camera.channel), NOW), "ok");
  assert.equal(ratesOf({ at: NOW, counts: [] }).size, 0, "unsubscribed channels disappear");
});

test("a subscribed channel that has not received data has unknown freshness", () => {
  const camera = GO2[1];
  const rates = ratesOf({ at: NOW, counts: [{ channel: camera.channel, hz: 0, ageS: null }] });
  const { groups } = systemGroups([camera], [], new Set(), rates, NOW);
  assert.equal(groups[0]?.cards[0]?.rate?.lastAt, null);
  assert.equal(groups[0]?.cards[0]?.level, null);
});

test("an authored relay channel has a preview without module inventory metadata", () => {
  const camera: StreamRef = { channel: "front-camera" as Channel, encoding: "jpeg.v1", source: null };
  const { groups, others } = systemGroups([camera], [], new Set(), new Map(), NOW);
  assert.equal(others, 0);
  assert.equal(groups[0]?.title, "Relay");
  assert.equal(groups[0]?.cards[0]?.title, "front-camera");
  assert.equal(groups[0]?.cards[0]?.picture, "image");
});
