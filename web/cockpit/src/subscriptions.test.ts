import { describe, expect, it } from "vitest";
import {
  type ChannelSpec,
  createDecoderRegistry,
  type Manifest,
  type PanelSpec,
  StatusStore,
  TRACK_ENCODING,
} from "@dimos/sdk";
import {
  channelSubscribable,
  installAutoSubscriptions,
  subscribableChannels,
} from "./subscriptions.ts";

function spec(over: Partial<ChannelSpec> = {}): ChannelSpec {
  return {
    ch: "odom",
    dir: "rx",
    encoding: "pose.json.v1",
    delivery: "reliable",
    maxHz: 20,
    params: {},
    publish: "none",
    requiredScope: null,
    ...over,
  };
}

function panel(over: Partial<PanelSpec> & { id: string; kind: string }): PanelSpec {
  return { title: "", channels: [], params: {}, ...over };
}

function mf(channels: ChannelSpec[], panels: PanelSpec[] = []): Manifest {
  return { version: 1, channels, panels, layout: null, pages: [] };
}

const odom = spec();
const jpeg = spec({ ch: "color_image", encoding: "jpeg.v1", delivery: "latest" });
const costmap = spec({ ch: "global_costmap", encoding: "costmap.zlib.v1", delivery: "latest" });
const track = spec({ ch: "color_image", encoding: TRACK_ENCODING, delivery: "latest" });
const future = spec({ ch: "voxels", encoding: "voxels.bin.v9", delivery: "latest" });
const lcm = spec({
  ch: "lcm_pose",
  encoding: "geometry_msgs.PoseStamped.lcm.v1",
  params: {
    lcm: { type: "t.P", fp: "0011223344556677", structs: { "t.P": [["x", "double", null]] } },
  },
});
const lcmBroken = spec({ ch: "lcm_bad", encoding: "t.Q.lcm.v1", params: {} });
// A variable-length array: every frame costs the message's full size.
const lcmCloud = spec({
  ch: "lcm_cloud",
  encoding: "sensor_msgs.PointCloud2.lcm.v1",
  params: {
    lcm: {
      type: "t.C",
      fp: "0011223344556677",
      structs: { "t.C": [["n", "int32_t", null], ["data", "byte", ["n"]]] },
    },
  },
});
const videoPanel = panel({ id: "cam", kind: "video", channels: ["color_image"] });
const mapPanel = panel({ id: "map", kind: "map2d", channels: ["global_costmap", "odom"] });
const voxels = spec({ ch: "global_map", encoding: "voxels.zlib.v1", delivery: "latest" });
const map3dPanel = panel({ id: "map3d", kind: "map3d", channels: ["global_map", "odom"] });

describe("subscribableChannels", () => {
  it("keeps only channels with a decoder (undecodable ones waste bandwidth)", () => {
    expect(subscribableChannels([odom, jpeg, future], [videoPanel])).toEqual([odom, jpeg]);
    expect(subscribableChannels([future], [])).toEqual([]);
  });

  it("subscribes bounded *.lcm.v1 schemas by itself, bulk ones only through a panel", () => {
    expect(channelSubscribable(lcm, [])).toBe(true);
    expect(channelSubscribable(lcm, [videoPanel])).toBe(true);
    expect(channelSubscribable(lcmCloud, [])).toBe(false);
    const cloudPanel = panel({ id: "cloud", kind: "video", channels: ["lcm_cloud"] });
    expect(channelSubscribable(lcmCloud, [cloudPanel])).toBe(true);
    // No schema in params: nothing can decode it, so nothing subscribes.
    expect(channelSubscribable(lcmBroken, [])).toBe(false);
    expect(subscribableChannels([lcm, lcmCloud, lcmBroken], [])).toEqual([lcm]);
  });

  it("never subscribes tx channels", () => {
    expect(channelSubscribable(spec({ dir: "tx" }), [])).toBe(false);
  });

  it("gates panel-only encodings on a renderable panel binding them", () => {
    expect(channelSubscribable(jpeg, [])).toBe(false);
    expect(channelSubscribable(jpeg, [videoPanel])).toBe(true);
    // A panel kind this build cannot render does not justify the bandwidth
    // (and the UnknownPanel fallback must never leak into this gate).
    expect(channelSubscribable(jpeg, [{ ...videoPanel, kind: "hologram" }])).toBe(false);
    // Cheap JSON channels are subscribed with or without a panel.
    expect(channelSubscribable(odom, [])).toBe(true);
  });

  it("gates the costmap encoding like jpeg (grids nobody renders stay unencoded)", () => {
    expect(channelSubscribable(costmap, [])).toBe(false);
    expect(channelSubscribable(costmap, [mapPanel])).toBe(true);
    expect(channelSubscribable(costmap, [{ ...mapPanel, kind: "hologram" }])).toBe(false);
  });

  it("gates the voxel map encoding like the costmap", () => {
    expect(channelSubscribable(voxels, [])).toBe(false);
    expect(channelSubscribable(voxels, [map3dPanel])).toBe(true);
    expect(channelSubscribable(voxels, [{ ...map3dPanel, kind: "hologram" }])).toBe(false);
  });

  it("gates track channels like jpeg (a pull nobody renders makes the robot encode)", () => {
    expect(channelSubscribable(track, [])).toBe(false);
    expect(channelSubscribable(track, [videoPanel])).toBe(true);
    expect(channelSubscribable(track, [{ ...videoPanel, kind: "hologram" }])).toBe(false);
    expect(subscribableChannels([odom, track], [videoPanel])).toEqual([odom, track]);
  });

  it("consults the given registry, not a global one", () => {
    const registry = createDecoderRegistry();
    registry.register("voxels.bin.v9", () => ({ value: null }));
    expect(channelSubscribable(future, [], registry)).toBe(true);
    expect(channelSubscribable(future, [])).toBe(false);
  });
});

describe("installAutoSubscriptions", () => {
  function harness() {
    const status = new StatusStore();
    const subscribed: string[] = [];
    const released: string[] = [];
    const session = {
      status,
      subscribe: (ch: string) => {
        subscribed.push(ch);
        return () => released.push(ch);
      },
    };
    return { status, session, subscribed, released };
  }

  it("acquires handles for subscribable channels on every adoption", () => {
    const { status, session, subscribed } = harness();
    const subs = installAutoSubscriptions(session);
    expect(subscribed).toEqual([]); // nothing before a manifest

    status.update({ manifest: mf([odom, jpeg]) }); // no panel binds the jpeg
    expect(subscribed).toEqual(["odom"]);

    status.update({ manifest: mf([odom, jpeg], [videoPanel]) });
    expect(subscribed).toEqual(["odom"]); // bound, but its panel is not on screen
    subs.setShownPanels(["cam"]);
    expect(subscribed).toEqual(["odom", "color_image"]);
  });

  it("holds panel-only channels only while a panel binding them is on screen", () => {
    const { status, session, subscribed, released } = harness();
    const subs = installAutoSubscriptions(session);
    status.update({ manifest: mf([odom, track], [videoPanel]) });
    subs.setShownPanels(["cam"]);
    expect(subscribed).toEqual(["odom", "color_image"]);

    subs.setShownPanels([]); // the channels tab, or another page
    expect(released).toEqual(["color_image"]);
    expect(subscribed).toEqual(["odom", "color_image"]); // odom stays held

    // A robot restart meanwhile: the shown set is remembered for the return.
    status.update({ manifest: null });
    subs.setShownPanels(["cam"]);
    expect(subscribed).toEqual(["odom", "color_image"]);
    status.update({ manifest: mf([odom, track], [videoPanel]) });
    expect(subscribed).toEqual(["odom", "color_image", "color_image"]);
    expect(released).toEqual(["color_image"]);
  });

  it("releases handles for channels the new manifest drops, keeping the rest", () => {
    const { status, session, subscribed, released } = harness();
    const subs = installAutoSubscriptions(session);
    subs.setShownPanels(["cam"]);
    status.update({ manifest: mf([odom, jpeg], [videoPanel]) });
    expect(subscribed).toEqual(["odom", "color_image"]);

    status.update({ manifest: mf([odom]) });
    expect(released).toEqual(["color_image"]);
    expect(subscribed).toEqual(["odom", "color_image"]); // odom held, not re-acquired
  });

  it("keeps handles while the manifest is null (robot restart reattaches)", () => {
    const { status, session, subscribed, released } = harness();
    installAutoSubscriptions(session);
    status.update({ manifest: mf([odom]) });
    status.update({ manifest: null });
    expect(released).toEqual([]);

    status.update({ manifest: mf([odom]) }); // fresh object, same content
    expect(released).toEqual([]);
    expect(subscribed).toEqual(["odom"]);
  });

  it("releases everything on dispose and stops reconciling", () => {
    const { status, session, subscribed, released } = harness();
    const subs = installAutoSubscriptions(session);
    status.update({ manifest: mf([odom]) });
    subs.dispose();
    expect(released).toEqual(["odom"]);

    status.update({ manifest: mf([odom, spec({ ch: "gps" })]) });
    expect(subscribed).toEqual(["odom"]);
  });
});
