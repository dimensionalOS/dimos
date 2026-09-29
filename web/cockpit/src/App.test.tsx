// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it, type MockInstance, vi } from "vitest";
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
const ROBOT_B = { id: "b", name: "B", model: "go2" };

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

// A *.lcm.v1 channel decodes from the schema in its params; one without a
// usable schema has no decoder at all.
const LCM_POSE: ChannelSpec = {
  ...ODOM,
  ch: "lcm_pose",
  encoding: "geometry_msgs.PoseStamped.lcm.v1",
  params: {
    lcm: { type: "t.P", fp: "0011223344556677", structs: { "t.P": [["x", "double", null]] } },
  },
};
const LCM_BROKEN: ChannelSpec = { ...ODOM, ch: "lcm_bad", encoding: "t.Q.lcm.v1" };

function mf(channels: ChannelSpec[], panels: PanelSpec[] = []): Manifest {
  return { version: 1, channels, panels, layout: null, pages: [] };
}

const CAM: PanelSpec = {
  id: "cam",
  kind: "video",
  title: "",
  channels: ["color_image"],
  params: {},
};

describe("App session states", () => {
  let container: HTMLElement;
  let root: Root;
  let status: StatusStore;
  let channels: ChannelStore;
  let session: Session;
  let watch: Session["watch"];

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    status = new StatusStore();
    channels = new ChannelStore();
    watch = vi.fn((_id: string) => new Promise<Manifest>(() => {}));
    session = {
      status,
      store: channels,
      watch,
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
  const picker = () => container.querySelector('[data-testid="robot-picker"]');
  const pickEntries = () => container.querySelectorAll('[data-testid^="robot-pick-"]');
  const pick = (id: string) => {
    act(() => container.querySelector<HTMLElement>(`[data-testid="robot-pick-${id}"]`)!.click());
  };
  const switchButton = () => container.querySelector<HTMLElement>('[data-testid="switch-robot"]');
  const panel = () => container.querySelector('[data-testid="panel-cam"]');

  it("shows *.lcm.v1 rows from their manifest schema, not a registered decoder", () => {
    act(() => {
      status.update({ watchedRobot: ROBOT, robots: [ROBOT] });
      status.update({ manifest: mf([LCM_POSE, LCM_BROKEN]) });
      channels.ingest(
        "lcm_pose",
        { ch: "lcm_pose", seq: 3, ts: 0.3, delivery: "reliable" },
        { x: 1.5 },
        true,
        "{x: 1.5}",
      );
      channels.publishUi();
    });
    view("channels");
    expect(container.querySelector('[data-testid="ch-lcm_pose-seq"]')!.textContent).toBe("3");
    expect(container.querySelector('[data-testid="ch-lcm_pose-value"]')!.textContent).toContain(
      "{x: 1.5}",
    );
    expect(container.querySelector('[data-testid="ch-lcm_bad-value"]')!.textContent).toContain(
      "no decoder for t.Q.lcm.v1",
    );
  });

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
      const testId = id === "overview" ? "tab-overview" : `tab-page-${id}`;
      act(() => container.querySelector<HTMLElement>(`[data-testid="${testId}"]`)!.click());
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
    const row = () => container.querySelector('[data-testid="ch-odom-seq"]');
    const selected = (v: View) =>
      container.querySelector(`[data-testid="view-${v}"]`)!.getAttribute("aria-selected");
    expect(panel()).not.toBeNull();
    expect(row()).toBeNull();
    expect(selected("panels")).toBe("true");
    // A lone robot is auto-watched: nothing to pick or switch to.
    expect(picker()).toBeNull();
    expect(switchButton()).toBeNull();

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

  it("keeps the open page across a manifest epoch and drops it when it vanishes", () => {
    const cam: PanelSpec = {
      id: "cam",
      kind: "video",
      title: "Front camera",
      channels: ["color_image"],
      params: {},
    };
    const withPage: Manifest = { ...mf([ODOM, IMAGE], [cam]), pages: ["cam"] };
    const tab = (id: string) => {
      const testId = id === "overview" ? "tab-overview" : `tab-page-${id}`;
      return container.querySelector(`[data-testid="${testId}"]`);
    };
    act(() => status.update({ watchedRobot: ROBOT, robots: [ROBOT], manifest: withPage }));
    expect(tab("overview")!.getAttribute("aria-selected")).toBe("true");
    act(() => (tab("cam") as HTMLElement).click());
    expect(tab("cam")!.getAttribute("aria-selected")).toBe("true");
    expect(container.querySelector('[data-testid="panel-cam"]')).not.toBeNull();

    // A robot restart (same manifest, new epoch) keeps the operator on the page.
    act(() => status.update({ manifest: { ...withPage }, epoch: 1 }));
    expect(tab("cam")!.getAttribute("aria-selected")).toBe("true");

    // The page is gone from the new manifest: back to the grid, no strip.
    act(() => status.update({ manifest: mf([ODOM, IMAGE], [cam]), epoch: 2 }));
    expect(tab("cam")).toBeNull();
    expect(container.querySelector('[data-testid="panel-cam"]')).not.toBeNull();

    // Reintroducing the page does not resurrect the discarded selection.
    act(() => status.update({ manifest: withPage, epoch: 3 }));
    expect(tab("overview")!.getAttribute("aria-selected")).toBe("true");
    expect(tab("cam")!.getAttribute("aria-selected")).toBe("false");
    expect(container.querySelector('[data-testid="panel-cam"]')).toBeNull();
  });

  it("leaves the channels view when a page tab is picked", () => {
    const cam: PanelSpec = {
      id: "cam",
      kind: "video",
      title: "",
      channels: ["color_image"],
      params: {},
    };
    const selected = (id: string) =>
      container.querySelector(`[data-testid="${id}"]`)!.getAttribute("aria-selected");
    act(() => {
      status.update({
        watchedRobot: ROBOT,
        robots: [ROBOT],
        manifest: { ...mf([ODOM, IMAGE], [cam]), pages: ["cam"] },
      });
    });
    view("channels");
    expect(selected("tab-overview")).toBe("false");
    expect(container.querySelector('[data-testid="ch-odom-seq"]')).not.toBeNull();

    act(() => container.querySelector<HTMLElement>('[data-testid="tab-page-cam"]')!.click());
    expect(selected("view-panels")).toBe("true");
    expect(selected("tab-page-cam")).toBe("true");
    expect(container.querySelector('[data-testid="ch-odom-seq"]')).toBeNull();
    expect(container.querySelector('[data-testid="panel-cam"]')).not.toBeNull();
  });

  it("watches the operator's pick and waits if that robot disappears", () => {
    act(() => status.update({ robots: [ROBOT, ROBOT_B] }));
    expect(picker()).not.toBeNull();
    expect(pickEntries()).toHaveLength(2);
    expect(switchButton()).toBeNull();
    expect(container.textContent).not.toContain("Waiting for a robot");

    pick("b");
    expect(watch).toHaveBeenCalledWith("b");
    act(() => status.update({ watchedRobot: ROBOT_B }));
    expect(picker()).toBeNull();
    expect(container.textContent).toContain("Waiting for a robot");
    expect(switchButton()).not.toBeNull();

    act(() => status.update({ manifest: mf([ODOM, IMAGE], [CAM]) }));
    expect(panel()).not.toBeNull();

    act(() => {
      status.update({ watchedRobot: null, robots: [ROBOT], manifest: null, epoch: 1 });
    });
    expect(picker()).toBeNull();
    expect(switchButton()).toBeNull();
    expect(container.textContent).toContain("Waiting for a robot");
  });

  it("reopens the picker on 'switch robot' with the watched robot marked", () => {
    act(() => {
      status.update({ watchedRobot: ROBOT, robots: [ROBOT, ROBOT_B], manifest: mf([ODOM], [CAM]) });
    });
    expect(picker()).toBeNull();
    act(() => switchButton()!.click());
    expect(picker()).not.toBeNull();
    expect(panel()).toBeNull();
    expect(switchButton()).toBeNull();
    const current = (id: string) =>
      container.querySelector(`[data-testid="robot-pick-${id}"]`)!.getAttribute("aria-current");
    expect(current("a")).toBe("true");
    expect(current("b")).toBe("false");

    // The other robot: the old producer is dropped until its manifest lands.
    pick("b");
    expect(watch).toHaveBeenCalledWith("b");
    act(() => status.update({ watchedRobot: ROBOT_B, manifest: null, epoch: 1 }));
    expect(picker()).toBeNull();
    expect(container.textContent).toContain("Waiting for a robot");
    act(() => status.update({ manifest: mf([ODOM, IMAGE], [CAM]) }));
    expect(panel()).not.toBeNull();
    expect(switchButton()).not.toBeNull();
  });

  it("drops a stale switch request when the other robot leaves", () => {
    act(() => {
      status.update({ watchedRobot: ROBOT, robots: [ROBOT, ROBOT_B], manifest: mf([ODOM], [CAM]) });
    });
    act(() => switchButton()!.click());
    expect(picker()).not.toBeNull();
    act(() => status.update({ robots: [ROBOT] }));
    expect(picker()).toBeNull();
    expect(panel()).not.toBeNull();
    // The other robot returning must not pop the picker over the layout.
    act(() => status.update({ robots: [ROBOT, ROBOT_B] }));
    expect(picker()).toBeNull();
    expect(switchButton()).not.toBeNull();
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

  describe("relay auth", () => {
    let reload: MockInstance<() => void>;

    beforeEach(() => {
      // happy-dom's reload navigates for real; the App only needs the call.
      reload = vi.spyOn(location, "reload").mockImplementation(() => {});
    });

    afterEach(() => {
      reload.mockRestore();
      localStorage.clear();
    });

    const authFailed = (reason: string) => {
      act(() => status.update({ transport: { phase: "failed", reason, code: "auth_failed" } }));
    };
    const logOut = () => container.querySelector<HTMLElement>('[data-testid="log-out"]');

    it("shows the token form for auth_failed with the relay's message", () => {
      authFailed("missing viewer token");
      expect(container.querySelector('[data-testid="token-message"]')?.textContent).toBe(
        "missing viewer token",
      );
      expect(container.textContent).not.toContain("Connection failed");
      expect(logOut()).toBeNull();
    });

    it("submitting the form stores the token and reloads", () => {
      authFailed("invalid viewer token");
      const input = container.querySelector<HTMLInputElement>('[data-testid="token-input"]')!;
      act(() => {
        // React tracks controlled inputs through the value setter; go around it.
        Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")?.set?.call(
          input,
          "tok-en",
        );
        input.dispatchEvent(new Event("input", { bubbles: true }));
      });
      act(() => {
        input.form?.dispatchEvent(new Event("submit", { bubbles: true, cancelable: true }));
      });
      expect(localStorage.getItem("dimos.cockpit.token")).toBe("tok-en");
      expect(reload).toHaveBeenCalledTimes(1);
    });

    it("offers 'log out' only with a stored token; it forgets the token and reloads", () => {
      localStorage.setItem("dimos.cockpit.token", "tok-en");
      act(() => root.render(<App session={session} />));
      act(() => logOut()!.click());
      expect(localStorage.getItem("dimos.cockpit.token")).toBeNull();
      expect(reload).toHaveBeenCalledTimes(1);
    });
  });
});
