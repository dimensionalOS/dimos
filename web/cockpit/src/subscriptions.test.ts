import { describe, expect, it } from "vitest";
import {
  type ChannelSpec,
  createDecoderRegistry,
  type Manifest,
  type PanelSpec,
  StatusStore,
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
const future = spec({ ch: "voxels", encoding: "voxels.bin.v9", delivery: "latest" });
const videoPanel = panel({ id: "cam", kind: "video", channels: ["color_image"] });
const mapPanel = panel({ id: "map", kind: "map2d", channels: ["global_costmap", "odom"] });

describe("subscribableChannels", () => {
  it("keeps only channels with a decoder (undecodable ones waste bandwidth)", () => {
    expect(subscribableChannels([odom, jpeg, future], [videoPanel])).toEqual([odom, jpeg]);
    expect(subscribableChannels([future], [])).toEqual([]);
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
    installAutoSubscriptions(session);
    expect(subscribed).toEqual([]); // nothing before a manifest

    status.update({ manifest: mf([odom, jpeg]) }); // no panel binds the jpeg
    expect(subscribed).toEqual(["odom"]);

    status.update({ manifest: mf([odom, jpeg], [videoPanel]) });
    expect(subscribed).toEqual(["odom", "color_image"]);
  });

  it("releases handles for channels the new manifest drops, keeping the rest", () => {
    const { status, session, subscribed, released } = harness();
    installAutoSubscriptions(session);
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
    const dispose = installAutoSubscriptions(session);
    status.update({ manifest: mf([odom]) });
    dispose();
    expect(released).toEqual(["odom"]);

    status.update({ manifest: mf([odom, spec({ ch: "gps" })]) });
    expect(subscribed).toEqual(["odom"]);
  });
});
