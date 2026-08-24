// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { ChannelSpec, PanelSpec } from "@dimos/shared";
import type { Manifest } from "@dimos/shared/manifest";
import { App } from "./App.tsx";
import type { SessionHandle } from "./session/session.ts";
import { ChannelStore, StatusStore } from "./session/store.ts";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

const ODOM: ChannelSpec = {
  ch: "odom",
  dir: "rx",
  encoding: "pose.json.v1",
  delivery: "reliable",
  maxHz: 20,
  params: {},
};
const IMAGE: ChannelSpec = {
  ch: "color_image",
  dir: "rx",
  encoding: "jpeg.v1",
  delivery: "latest",
  maxHz: 15,
  params: {},
};

function mf(channels: ChannelSpec[], panels: PanelSpec[] = []): Manifest {
  return { version: 1, channels, panels, layout: null, pages: [] };
}

describe("App session states", () => {
  let container: HTMLElement;
  let root: Root;
  let status: StatusStore;
  let channels: ChannelStore;
  let session: SessionHandle;

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    status = new StatusStore();
    channels = new ChannelStore();
    session = { status, channels, stop: () => {} };
    act(() => root.render(<App session={session} />));
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
  });

  it("waits for a robot, shows its channels, and clears them when it leaves", () => {
    expect(container.textContent).toContain("Waiting for a robot");

    act(() => {
      status.update({ robot: { id: "a", name: "A", model: "go2" }, robotCount: 1 });
      status.update({ manifest: mf([ODOM]) });
      channels.ingest(
        "odom",
        { ch: "odom", seq: 7, ts: 0.7, delivery: "reliable" },
        { x: 1 },
        true,
        '{"x":1}',
      );
      channels.publishUi();
    });
    expect(container.querySelector('[data-testid="ch-odom-seq"]')!.textContent).toBe("7");
    expect(container.querySelector('[data-testid="ch-odom-value"]')!.textContent).toContain(
      '{"x":1}',
    );

    // Robot gone: the session clears the manifest and bumps the epoch; the
    // list must unmount instead of keeping stale rows.
    act(() => {
      channels.reset();
      status.update({ robot: null, robotCount: 0, manifest: null, epoch: 1 });
    });
    expect(container.querySelector('[data-testid="ch-odom-seq"]')).toBeNull();
    expect(container.textContent).toContain("Waiting for a robot");
  });

  it("keeps the last good frame on decode failures and flags them visibly", () => {
    act(() => {
      status.update({ robot: { id: "a", name: "A", model: "go2" }, robotCount: 1 });
      status.update({ manifest: mf([ODOM]) });
      channels.ingest(
        "odom",
        { ch: "odom", seq: 7, ts: 0.7, delivery: "reliable" },
        { x: 1 },
        true,
        '{"x":1}',
      );
      channels.publishUi();
    });
    expect(container.querySelector('[data-testid="ch-odom-decode-error"]')).toBeNull();

    // Corrupt frames arrive: the row keeps describing the good frame (seq and
    // value stay together) and a decode-error indicator appears.
    act(() => {
      channels.ingest(
        "odom",
        { ch: "odom", seq: 8, ts: 0.8, delivery: "reliable" },
        undefined,
        false,
      );
      channels.publishUi();
    });
    expect(container.querySelector('[data-testid="ch-odom-seq"]')!.textContent).toBe("7");
    expect(container.querySelector('[data-testid="ch-odom-value"]')!.textContent).toContain(
      '{"x":1}',
    );
    expect(
      container.querySelector('[data-testid="ch-odom-decode-error"]')!.textContent,
    ).toContain("decode failing");

    // Recovery: the next good frame clears the indicator and advances the row.
    act(() => {
      channels.ingest(
        "odom",
        { ch: "odom", seq: 9, ts: 0.9, delivery: "reliable" },
        { x: 2 },
        true,
        '{"x":2}',
      );
      channels.publishUi();
    });
    expect(container.querySelector('[data-testid="ch-odom-decode-error"]')).toBeNull();
    expect(container.querySelector('[data-testid="ch-odom-seq"]')!.textContent).toBe("9");
  });

  it("marks the jpeg channel unsubscribed until a video panel binds it", () => {
    act(() => {
      status.update({
        robot: { id: "a", name: "A", model: "go2" },
        robotCount: 1,
        manifest: mf([ODOM, IMAGE]),
      });
    });
    const value = () => container.querySelector('[data-testid="ch-color_image-value"]')!;
    expect(value().textContent).toContain("not subscribed (no panel binds it)");

    // The manifest gains a video panel: the session subscribes, the row waits.
    act(() => {
      status.update({
        manifest: mf([ODOM, IMAGE], [
          { id: "cam", kind: "video", title: "", channels: ["color_image"], params: {} },
        ]),
      });
    });
    expect(value().textContent).toContain("waiting for data...");
  });

  it("shows the multi-robot notice instead of channels", () => {
    act(() => status.update({ robotCount: 2 }));
    expect(container.textContent).toContain("2 robots connected");
  });

  it("shows the polite notice on an unsupported manifest version", () => {
    act(() => {
      status.update({
        robot: { id: "a", name: "A", model: "go2" },
        robotCount: 1,
        manifestUnsupported: true,
      });
    });
    expect(container.textContent).toContain("newer than this Cockpit build");
    expect(container.querySelector('[data-testid="ch-odom-seq"]')).toBeNull();
  });

  it("shows the terminal failure reason", () => {
    act(() => status.update({ transport: { phase: "failed", reason: "protocol mismatch" } }));
    expect(container.textContent).toContain("Connection failed: protocol mismatch");
  });
});
