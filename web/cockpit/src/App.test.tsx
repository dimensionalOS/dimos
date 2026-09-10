// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import { ChannelStore, type Session, StatusStore } from "@dimos/sdk";
import { registerTeleopHooks } from "@dimos/sdk/internal/teleop";
import type { ChannelSpec, PanelSpec } from "@dimos/shared";
import type { Manifest } from "@dimos/shared/manifest";
import { App } from "./App.tsx";
import type { View } from "./ui/StatusBar.tsx";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

const ROBOT = { id: "a", name: "A", model: "go2" };

const ODOM: ChannelSpec = {
  ch: "odom",
  dir: "rx",
  encoding: "pose.json.v1",
  delivery: "reliable",
  maxHz: 20,
  params: {},
  publish: "none",
  requiredScope: null,
};
const IMAGE: ChannelSpec = {
  ch: "color_image",
  dir: "rx",
  encoding: "jpeg.v1",
  delivery: "latest",
  maxHz: 15,
  params: {},
  publish: "none",
  requiredScope: null,
};

function mf(channels: ChannelSpec[], panels: PanelSpec[] = []): Manifest {
  return { version: 1, channels, panels, layout: null, pages: [] };
}

describe("App session states", () => {
  let container: HTMLElement;
  let root: Root;
  let status: StatusStore;
  let channels: ChannelStore;
  let session: Session;

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    status = new StatusStore();
    channels = new ChannelStore();
    session = {
      status,
      store: channels,
      watch: () => new Promise(() => {}),
      subscribe: () => () => {},
      publish: () => new Promise(() => {}),
      close: () => {},
    };
    registerTeleopHooks(session, {
      control: () => {},
      datagram: () => {},
      onMsg: () => () => {},
      status,
    });
    act(() => root.render(<App session={session} />));
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
  });

  const view = (v: View) => {
    act(() => container.querySelector<HTMLElement>(`[data-testid="view-${v}"]`)!.click());
  };

  it("waits for a robot, shows its channels, and clears them when it leaves", () => {
    expect(container.textContent).toContain("Waiting for a robot");

    act(() => {
      status.update({ watchedRobot: ROBOT, robots: [ROBOT] });
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
    view("channels");
    expect(container.querySelector('[data-testid="ch-odom-seq"]')!.textContent).toBe("7");
    expect(container.querySelector('[data-testid="ch-odom-value"]')!.textContent).toContain(
      '{"x":1}',
    );

    // Robot gone: the session clears the manifest and bumps the epoch; the
    // list must unmount instead of keeping stale rows.
    act(() => {
      channels.reset();
      status.update({ watchedRobot: null, robots: [], manifest: null, epoch: 1 });
    });
    expect(container.querySelector('[data-testid="ch-odom-seq"]')).toBeNull();
    expect(container.textContent).toContain("Waiting for a robot");
  });

  it("keeps the last good frame on decode failures and flags them visibly", () => {
    act(() => {
      status.update({ watchedRobot: ROBOT, robots: [ROBOT] });
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
    view("channels");
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
        watchedRobot: ROBOT,
        robots: [ROBOT],
        manifest: mf([ODOM, IMAGE]),
      });
    });
    view("channels");
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

  it("keeps a chat page's transcript since join across tab switches", () => {
    const TEXT: ChannelSpec = {
      ch: "human_input",
      dir: "tx",
      encoding: "text.json.v1",
      delivery: "reliable",
      maxHz: 5,
      params: {},
      publish: "shared",
      requiredScope: null,
    };
    const AGENT: ChannelSpec = { ...ODOM, ch: "agent", encoding: "chat.json.v1" };
    const IDLE: ChannelSpec = {
      ...ODOM,
      ch: "agent_idle",
      encoding: "json.v1",
      delivery: "latest",
    };
    const AUDIO: ChannelSpec = { ...TEXT, ch: "audio_in", encoding: "audio.json.v1", maxHz: 20 };
    const chat: PanelSpec = {
      id: "chat",
      kind: "chat",
      title: "",
      channels: ["human_input", "agent", "agent_idle", "audio_in"],
      params: {},
    };
    const line = (seq: number, text: string) => {
      channels.ingest(
        "agent",
        { ch: "agent", seq, ts: seq, delivery: "reliable" },
        [{ role: "ai", text, ts: seq }],
        true,
      );
    };
    const tab = (id: string) => {
      act(() => container.querySelector<HTMLElement>(`[data-testid="tab-${id}"]`)!.click());
    };
    act(() => {
      status.update({
        watchedRobot: ROBOT,
        robots: [ROBOT],
        manifest: { ...mf([ODOM, TEXT, AGENT, IDLE, AUDIO], [chat]), pages: ["chat"] },
      });
    });
    // Frames before the page is first opened, and while another tab shows.
    act(() => {
      line(1, "one");
      line(2, "two");
    });
    tab("chat");
    expect(container.querySelectorAll("[data-role]")).toHaveLength(2);
    tab("overview");
    act(() => line(3, "three"));
    tab("chat");
    const lines = [...container.querySelectorAll("[data-role]")].map((el) => el.textContent);
    expect(lines).toHaveLength(3);
    expect(lines.join(" ")).toContain("one");
    expect(lines.join(" ")).toContain("three");
  });

  it("shows the panels by default and the channel table on the channels tab", () => {
    const cam: PanelSpec = {
      id: "cam",
      kind: "video",
      title: "",
      channels: ["color_image"],
      params: {},
    };
    act(() => {
      status.update({ watchedRobot: ROBOT, robots: [ROBOT], manifest: mf([ODOM, IMAGE], [cam]) });
    });
    const panel = () => container.querySelector('[data-testid="panel-cam"]');
    const row = () => container.querySelector('[data-testid="ch-odom-seq"]');
    const selected = (v: View) =>
      container.querySelector(`[data-testid="view-${v}"]`)!.getAttribute("aria-selected");
    expect(panel()).not.toBeNull();
    expect(row()).toBeNull();
    expect(selected("panels")).toBe("true");

    view("channels");
    expect(selected("channels")).toBe("true");
    expect(panel()).toBeNull();
    expect(row()).not.toBeNull();

    // A manifest change remounts <main> but keeps the operator's view.
    act(() => status.update({ manifest: mf([ODOM, IMAGE]), epoch: 1 }));
    expect(row()).not.toBeNull();

    // cockpit(channels=[...]) alone has nothing to lay out.
    view("panels");
    expect(row()).toBeNull();
    expect(container.textContent).toContain("no panels");
  });

  it("shows the multi-robot notice instead of channels", () => {
    act(() => status.update({ robots: [ROBOT, { id: "b", name: "B", model: "go2" }] }));
    expect(container.textContent).toContain("2 robots connected");
  });

  it("shows the polite notice on an unsupported manifest version", () => {
    act(() => {
      status.update({
        watchedRobot: ROBOT,
        robots: [ROBOT],
        manifestUnsupported: true,
      });
    });
    expect(container.textContent).toContain("newer than this Cockpit build");
    view("channels");
    expect(container.querySelector('[data-testid="ch-odom-seq"]')).toBeNull();
  });

  it("shows the terminal failure reason", () => {
    act(() => status.update({ transport: { phase: "failed", reason: "protocol mismatch" } }));
    expect(container.textContent).toContain("Connection failed: protocol mismatch");
  });
});
