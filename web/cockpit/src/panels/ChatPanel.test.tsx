// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { FrameHeader, Msg, PanelSpec } from "@dimos/shared";
import type { Manifest } from "@dimos/shared/manifest";
import { ChannelStore, PublishError, type Session, StatusStore } from "@dimos/sdk";
import type { TeleopHooks } from "@dimos/sdk/internal/teleop";
import { ChatPanel } from "./ChatPanel.tsx";
import { TeleopPanel } from "./TeleopPanel.tsx";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

const SPEC: PanelSpec = {
  id: "p0",
  kind: "chat",
  title: "",
  channels: ["human_input", "agent", "agent_idle"],
  params: {},
};
const TELEOP_SPEC: PanelSpec = {
  id: "p1",
  kind: "teleop",
  title: "",
  channels: ["tele_cmd_vel"],
  params: {},
};
const MANIFEST: Manifest = {
  version: 1,
  channels: [
    {
      ch: "tele_cmd_vel",
      dir: "tx",
      encoding: "twist.json.v1",
      delivery: "latest",
      maxHz: 15,
      params: { maxLinear: 0.8, maxAngular: 1.0, boost: 2.0, watchdogMs: 300 },
      publish: "none",
      requiredScope: null,
    },
  ],
  panels: [SPEC, TELEOP_SPEC],
  layout: { row: ["p0", "p1"] },
  pages: [],
};

function header(ch: string, seq: number): FrameHeader {
  return { ch, seq, ts: 1_700_000_000 + seq, delivery: ch === "agent" ? "reliable" : "latest" };
}

class FakeSession implements Session {
  status = new StatusStore();
  store = new ChannelStore();
  published: [string, unknown][] = [];
  reject: PublishError | null = null;
  watch = () => new Promise<never>(() => {});
  subscribe = () => () => {};
  publish = (ch: string, value: unknown) => {
    this.published.push([ch, value]);
    return this.reject === null
      ? Promise.resolve({ ch, relayTs: 1, bridgeTs: 2 })
      : Promise.reject(this.reject);
  };
  close = () => {};
}

class FakeHooks implements TeleopHooks {
  controls: Msg[] = [];
  datagrams: Msg[] = [];
  cbs = new Set<(msg: Msg) => void>();
  constructor(readonly status: StatusStore) {}
  control = (msg: Msg) => this.controls.push(msg);
  datagram = (msg: Msg) => this.datagrams.push(msg);
  onMsg = (cb: (msg: Msg) => void) => {
    this.cbs.add(cb);
    return () => this.cbs.delete(cb);
  };
  reply(msg: Msg): void {
    for (const cb of this.cbs) cb(msg);
  }
}

// React tracks controlled inputs through the value setter; go around it.
const setValue = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")?.set;

describe("ChatPanel", () => {
  let container: HTMLElement;
  let root: Root;
  let session: FakeSession;

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    session = new FakeSession();
    session.status.update({ transport: { phase: "connected" }, manifest: MANIFEST });
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
  });

  function mount(): void {
    act(() => root.render(<ChatPanel spec={SPEC} store={session.store} session={session} />));
  }

  function frame(ch: string, seq: number, value: unknown): void {
    act(() => {
      session.store.ingest(ch, header(ch, seq), value, true);
      session.store.publishUi();
    });
  }

  function find<T extends HTMLElement>(testId: string): T {
    const el = container.querySelector<T>(`[data-testid="${testId}"]`);
    if (el === null) throw new Error(`${testId} not rendered`);
    return el;
  }

  function roles(): string[] {
    return [...container.querySelectorAll<HTMLElement>("[data-role]")].map((el) =>
      el.dataset.role ?? ""
    );
  }

  function type(text: string): void {
    act(() => {
      const input = find<HTMLInputElement>("chat-human_input-input");
      setValue?.call(input, text);
      input.dispatchEvent(new Event("input", { bubbles: true }));
    });
  }

  function submit(): void {
    act(() => {
      find<HTMLInputElement>("chat-human_input-input").form?.dispatchEvent(
        new Event("submit", { bubbles: true, cancelable: true }),
      );
    });
  }

  it("accumulates every frame in order and renders each role", () => {
    frame("agent", 1, [{ role: "human", text: "walk forward", ts: 1 }]); // predates the mount
    mount();
    frame("agent", 2, [
      { role: "ai", text: "sure", ts: 2 },
      { role: "ai", tool: "nav", text: 'nav({"x":1})', ts: 2 },
    ]);
    frame("agent", 3, [{ role: "tool", tool: "nav", text: "arrived", ts: 3 }]);
    frame("agent", 4, [{ role: "function", text: "legacy", ts: 4 }]);
    expect(roles()).toEqual(["human", "ai", "ai", "tool", "function"]);
    expect(container.textContent).toContain('▶ nav({"x":1})');
    expect(container.textContent).toContain("↳ nav: arrived");
    expect(container.textContent).toContain("[function]");

    frame("agent", 5, "not an array of lines");
    frame("agent", 6, [{ role: "ai", text: "no timestamp" }]);
    expect(container.textContent).toContain("unreadable message");
    expect(container.textContent).not.toContain("no timestamp");

    act(() => session.store.reset());
    expect(roles()).toEqual([]);
    expect(container.textContent).toContain("no messages yet");
  });

  it("shows the thinking spinner while the agent is busy", () => {
    mount();
    expect(container.querySelector('[data-testid="chat-agent-thinking"]')).toBeNull();
    frame("agent_idle", 1, false);
    expect(container.querySelector('[data-testid="chat-agent-thinking"]')).not.toBeNull();
    frame("agent_idle", 2, true);
    expect(container.querySelector('[data-testid="chat-agent-thinking"]')).toBeNull();
  });

  it("publishes the draft and reports a rejection", async () => {
    mount();
    type("walk forward");
    submit();
    expect(session.published).toEqual([["human_input", "walk forward"]]);
    expect(find<HTMLInputElement>("chat-human_input-input").value).toBe("");
    // The echo, not the submit, puts the message in the transcript.
    expect(roles()).toEqual([]);
    await act(async () => {});

    session.reject = new PublishError("rejected", "rate_limited", "slow down");
    type("again");
    submit();
    await act(async () => {});
    expect(container.textContent).toContain("send failed: rate_limited: slow down");
    expect(find<HTMLInputElement>("chat-human_input-input").value).toBe("again");
  });

  it("disables the input while disconnected", () => {
    session.status.update({ transport: { phase: "reconnecting", attempt: 1, retryAtMs: 0 } });
    mount();
    expect(find<HTMLInputElement>("chat-human_input-input").disabled).toBe(true);
    type("hello");
    submit();
    expect(session.published).toEqual([]);
  });

  it("renders a notice without a send path", () => {
    act(() => root.render(<ChatPanel spec={SPEC} store={session.store} />));
    expect(container.textContent).toContain("no send path bound");
  });

  it("typing in the chat never drives the robot", () => {
    const hooks = new FakeHooks(session.status);
    act(() =>
      root.render(
        <div>
          <TeleopPanel spec={TELEOP_SPEC} store={session.store} teleop={hooks} />
          <ChatPanel spec={SPEC} store={session.store} session={session} />
        </div>,
      )
    );
    const pad = find<HTMLElement>("teleop-tele_cmd_vel");
    act(() => pad.focus());
    act(() => hooks.reply({ t: "teleop_started" }));
    expect(pad.dataset.state).toBe("armed");

    // Focusing the chat input disarms teleop (its one zero twist), and
    // keys typed there never reach the pad.
    act(() => find<HTMLInputElement>("chat-human_input-input").focus());
    expect(pad.dataset.state).toBe("disarmed");
    const sent = hooks.datagrams.length;
    act(() => {
      find<HTMLInputElement>("chat-human_input-input").dispatchEvent(
        new KeyboardEvent("keydown", { code: "KeyW", key: "w", bubbles: true, cancelable: true }),
      );
    });
    expect(hooks.datagrams.length).toBe(sent);
    expect(pad.dataset.state).toBe("disarmed");
  });
});
