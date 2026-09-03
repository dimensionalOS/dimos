// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { FrameHeader, Msg, PanelSpec } from "@dimos/shared";
import { ChannelStore, StatusStore } from "@dimos/sdk";
import type { TeleopHooks, TxResult } from "@dimos/sdk/internal/teleop";
import { AGENT_MODE_NOTICE, ChatPanel, EMPTY_TEXT, THINKING_TEXT } from "./ChatPanel.tsx";
import { ControlPanel } from "./ControlPanel.tsx";
import { PENDING_MS } from "./controlPolicy.ts";
import { NavMapPanel, NOTE_LINGER_MS, startNavOverlay } from "./NavMapPanel.tsx";
import { getPanel } from "./registry.tsx";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

// Panel specs as dimos/web/cockpit.py emits them (web/shared/fixtures/manifests.json,
// vector cockpit_microduck).
const CHAT_SPEC: PanelSpec = {
  id: "p5",
  kind: "chat",
  title: "Agent",
  channels: ["agent", "agent_idle", "mode", "human_input"],
  params: { chat: "agent", idle: "agent_idle", mode: "mode", input: "human_input" },
};
const CONTROL_SPEC: PanelSpec = {
  id: "p0",
  kind: "control",
  title: "",
  channels: ["mode", "policy_state", "nav_state", "ui_command"],
  params: { mode: "mode", policies: "policy_state", navState: "nav_state", command: "ui_command" },
};
const NAVMAP_SPEC: PanelSpec = {
  id: "p2",
  kind: "navmap",
  title: "Nav map",
  channels: ["global_costmap", "odom", "path", "places", "nav_state", "goal_request", "ui_command"],
  params: {
    costmap: "global_costmap",
    pose: "odom",
    path: "path",
    places: "places",
    navState: "nav_state",
    goal: "goal_request",
    command: "ui_command",
  },
};

class FakeHooks implements TeleopHooks {
  sent: { ch: string; data: Record<string, unknown> }[] = [];
  result: TxResult = { ok: true, seq: 1 };
  status = new StatusStore();
  control = (_msg: Msg) => {};
  datagram = (_msg: Msg) => {};
  onMsg = (_cb: (msg: Msg) => void) => () => {};
  tx = (ch: string, data: Record<string, unknown>): TxResult => {
    this.sent.push({ ch, data });
    return this.result;
  };
}

let seqs: Record<string, number> = {};
const now = 1_700_000_000_000;

function header(ch: string): FrameHeader {
  seqs[ch] = (seqs[ch] ?? 0) + 1;
  return { ch, seq: seqs[ch], ts: now / 1000, delivery: "reliable" };
}

function ingest(store: ChannelStore, ch: string, value: unknown): void {
  act(() => {
    store.ingest(ch, header(ch), value, true);
    store.publishUi();
  });
}

/** React's controlled inputs track the value setter; go through the prototype. */
function typeInto(el: HTMLTextAreaElement, text: string): void {
  const setter = Object.getOwnPropertyDescriptor(HTMLTextAreaElement.prototype, "value")?.set;
  act(() => {
    setter?.call(el, text);
    el.dispatchEvent(new Event("input", { bubbles: true }));
  });
}

function key(el: Element, type: "keydown" | "keyup", init: KeyboardEventInit): KeyboardEvent {
  const ev = new KeyboardEvent(type, { bubbles: true, cancelable: true, ...init });
  act(() => {
    el.dispatchEvent(ev);
  });
  return ev;
}

function click(el: Element, init: MouseEventInit = {}): void {
  act(() => {
    el.dispatchEvent(new MouseEvent("click", { bubbles: true, cancelable: true, ...init }));
  });
}

function mouseDownIsPrevented(el: Element): boolean {
  const ev = new MouseEvent("mousedown", { bubbles: true, cancelable: true });
  act(() => {
    el.dispatchEvent(ev);
  });
  return ev.defaultPrevented;
}

/** happy-dom has no layout; pin the CSS size the sinks read. */
function defineSize(el: HTMLElement, w: number, h: number): void {
  Object.defineProperty(el, "clientWidth", { configurable: true, value: w });
  Object.defineProperty(el, "clientHeight", { configurable: true, value: h });
}

describe("registry", () => {
  it("has the microduck cockpit kinds registered", () => {
    expect(getPanel("chat")).toBe(ChatPanel);
    expect(getPanel("navmap")).toBe(NavMapPanel);
    expect(getPanel("control")).toBe(ControlPanel);
  });
});

describe("cockpit panels", () => {
  let container: HTMLElement;
  let root: Root;
  let store: ChannelStore;
  let hooks: FakeHooks;
  let chatN: number;

  const q = <T extends HTMLElement>(testId: string): T => {
    const el = container.querySelector<T>(`[data-testid="${testId}"]`);
    if (el === null) throw new Error(`missing [data-testid="${testId}"]`);
    return el;
  };

  function chat(role: string, content: string, extra: Record<string, unknown> = {}): void {
    chatN += 1;
    ingest(store, "agent", {
      n: chatN,
      role,
      content,
      name: null,
      tool_calls: [],
      tool_call_id: null,
      id: null,
      t: now / 1000,
      ...extra,
    });
  }

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    // A fresh store per test keeps the module-level chat logs apart.
    store = new ChannelStore(() => now);
    hooks = new FakeHooks();
    seqs = {};
    chatN = 0;
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
    vi.restoreAllMocks();
    vi.useRealTimers();
  });

  describe("ChatPanel", () => {
    beforeEach(() => {
      act(() => root.render(<ChatPanel spec={CHAT_SPEC} store={store} teleop={hooks} />));
    });

    it("renders the humancli transcript from chat frames, with tool rows and the spinner", () => {
      // An empty transcript invites a message; it must not look like the
      // agent is still starting up.
      expect(container.textContent).toContain(EMPTY_TEXT);
      expect(container.textContent).not.toContain("waiting for");
      chat("human", "go to the kitchen");
      chat("ai", "on my way", {
        tool_calls: [{ id: "c1", name: "go_to", args: { room: "kitchen" } }],
      });
      chat("tool", "arrived", { tool_call_id: "c1" });
      const rows = Array.from(container.querySelectorAll<HTMLElement>("[data-kind]"));
      expect(rows.map((r) => r.dataset.kind)).toEqual(["human", "agent", "tool", "tool_result"]);
      const text = (r: HTMLElement) => r.querySelector("span:last-child")?.textContent;
      expect(rows.map(text)).toEqual([
        "go to the kitchen",
        "on my way",
        '▶ go_to({"room":"kitchen"})',
        "↳ arrived",
      ]);
      const prefix = rows[1].querySelector("span")?.textContent ?? "";
      expect(prefix).toMatch(/^ \d\d:\d\d:\d\d {4}agent $/);

      ingest(store, "agent_idle", { value: false, t: 1 });
      expect(q("chat-thinking").textContent).toContain(THINKING_TEXT);
      ingest(store, "agent_idle", { value: true, t: 2 });
      expect(container.querySelector('[data-testid="chat-thinking"]')).toBeNull();
    });

    it("sends on Enter, keeps Shift+Enter local, and never lets keys escape the box", () => {
      const input = q<HTMLTextAreaElement>("chat-agent-input");
      const leaked = vi.fn();
      document.addEventListener("keydown", leaked);
      document.addEventListener("keyup", leaked);
      try {
        typeInto(input, "  hello duck  ");
        key(input, "keydown", { key: "w", code: "KeyW" });
        key(input, "keyup", { key: "w", code: "KeyW" });
        expect(hooks.sent).toEqual([]);

        const shiftEnter = key(input, "keydown", { key: "Enter", shiftKey: true });
        expect(shiftEnter.defaultPrevented).toBe(false);
        expect(hooks.sent).toEqual([]);

        const enter = key(input, "keydown", { key: "Enter" });
        expect(enter.defaultPrevented).toBe(true);
        expect(hooks.sent).toEqual([{ ch: "human_input", data: { text: "hello duck" } }]);
        expect(input.value).toBe("");
        expect(leaked).not.toHaveBeenCalled();
      } finally {
        document.removeEventListener("keydown", leaked);
        document.removeEventListener("keyup", leaked);
      }

      const pending = q("chat-pending");
      expect(pending.dataset.status).toBe("pending");
      expect(pending.textContent).toContain("hello duck");
      // The agent loop echoes the HumanMessage: pending settles into a row.
      chat("human", "hello duck");
      expect(container.querySelector('[data-testid="chat-pending"]')).toBeNull();
      expect(container.querySelectorAll('[data-kind="human"]')).toHaveLength(1);
    });

    it("flags an unacknowledged line after 5 s and retries through tx", () => {
      // The log binds Date.now when it is created: fake the clock before mounting
      // a fresh store (chatLogFor caches one log per store and channel).
      vi.useFakeTimers();
      act(() => root.unmount());
      store = new ChannelStore(() => now);
      root = createRoot(container);
      act(() => root.render(<ChatPanel spec={CHAT_SPEC} store={store} teleop={hooks} />));
      const input = q<HTMLTextAreaElement>("chat-agent-input");
      typeInto(input, "are you there?");
      key(input, "keydown", { key: "Enter" });
      act(() => {
        vi.advanceTimersByTime(5000);
      });
      const pending = q("chat-pending");
      expect(pending.dataset.status).toBe("failed");
      expect(pending.textContent).toContain("not delivered");
      const retry = pending.querySelector("button");
      expect(retry).not.toBeNull();
      expect(mouseDownIsPrevented(retry as HTMLElement)).toBe(true);
      click(retry as HTMLElement);
      expect(hooks.sent).toHaveLength(2);
      expect(hooks.sent[1]).toEqual({ ch: "human_input", data: { text: "are you there?" } });
      expect(q("chat-pending").dataset.status).toBe("pending");
    });

    it("reports a refused send and blocks over-long lines", () => {
      const input = q<HTMLTextAreaElement>("chat-agent-input");
      hooks.result = { ok: false, reason: "disconnected" };
      typeInto(input, "hi");
      key(input, "keydown", { key: "Enter" });
      expect(q("chat-agent-error").textContent).toBe("not connected");
      expect(input.value).toBe("hi"); // the draft survives a failed send
      expect(container.querySelector('[data-testid="chat-pending"]')).toBeNull();

      hooks.result = { ok: true, seq: 2 };
      typeInto(input, "x".repeat(901));
      key(input, "keydown", { key: "Enter" });
      expect(hooks.sent).toHaveLength(1);
      expect(q("chat-agent-error").textContent).toContain("too long");
    });

    it("asks for agent mode instead of the transcript while the duck is in teleop", () => {
      chat("ai", "hello");
      ingest(store, "mode", { mode: "teleop", t: 1 });
      expect(q("chat-agent-notice").textContent).toBe(AGENT_MODE_NOTICE);
      expect(container.querySelector('[data-testid="chat-agent-input"]')).toBeNull();
      ingest(store, "mode", { mode: "agent", t: 2 });
      expect(container.querySelector('[data-testid="chat-agent-notice"]')).toBeNull();
      expect(container.textContent).toContain("hello"); // the log kept the rows
    });

    it("renders visibly with nothing bound and disables input without a send path", () => {
      act(() => root.unmount());
      root = createRoot(container);
      const bare: PanelSpec = { id: "c0", kind: "chat", title: "", channels: [], params: {} };
      act(() => root.render(<ChatPanel spec={bare} store={store} />));
      expect(container.textContent).toContain("no channel bound");
      act(() => root.unmount());
      root = createRoot(container);
      act(() => root.render(<ChatPanel spec={CHAT_SPEC} store={store} />));
      expect(q<HTMLTextAreaElement>("chat-agent-input").disabled).toBe(true);
    });
  });

  describe("ControlPanel", () => {
    const POLICY_STATE = {
      variant: "default",
      active: "walk",
      base: "walk",
      seated: false,
      fallen: false,
      locked: false,
      oneshot: null,
      policies: [
        { name: "walk", kind: "base", available: true, reason: null },
        { name: "stand", kind: "base", available: true, reason: null },
        { name: "sitstand", kind: "posture", available: true, reason: null },
        { name: "kick_left", kind: "oneshot", available: false, reason: "not this variant" },
        { name: "roulade", kind: "oneshot", available: true, reason: null },
      ],
      last_error: null,
      t: 1,
    };

    beforeEach(() => {
      act(() => root.render(<ControlPanel spec={CONTROL_SPEC} store={store} teleop={hooks} />));
    });

    it("switches mode optimistically and sends set_mode", () => {
      ingest(store, "mode", { mode: "teleop", t: 1 });
      const teleop = q<HTMLButtonElement>("control-mode-teleop");
      const agent = q<HTMLButtonElement>("control-mode-agent");
      expect(teleop.getAttribute("aria-pressed")).toBe("true");
      expect(mouseDownIsPrevented(agent)).toBe(true);
      click(agent);
      expect(hooks.sent).toEqual([
        { ch: "ui_command", data: { name: "set_mode", args: { mode: "agent" } } },
      ]);
      expect(agent.getAttribute("aria-pressed")).toBe("true");
      expect(agent.dataset.pending).toBe("true");
      ingest(store, "mode", { mode: "agent", t: 2 });
      expect(agent.dataset.pending).toBeUndefined();
      expect(teleop.getAttribute("aria-pressed")).toBe("false");
    });

    it("renders grouped policy buttons from policy_state and sends policy commands", () => {
      expect(container.textContent).toContain("waiting for policy state...");
      ingest(store, "policy_state", POLICY_STATE);
      const groups = Array.from(container.querySelectorAll<HTMLElement>("[role=group]"))
        .map((g) => g.getAttribute("aria-label"));
      expect(groups).toEqual(["mode", "Base", "Posture", "Actions"]);
      expect(q("policy-walk").dataset.state).toBe("active");
      expect(q("policy-stand").dataset.state).toBe("idle");
      const kick = q<HTMLButtonElement>("policy-kick_left");
      expect(kick.dataset.state).toBe("unavailable");
      expect(kick.disabled).toBe(true);
      expect(kick.title).toBe("not this variant");

      const roulade = q<HTMLButtonElement>("policy-roulade");
      expect(mouseDownIsPrevented(roulade)).toBe(true);
      click(roulade);
      expect(hooks.sent).toEqual([
        {
          ch: "ui_command",
          data: { name: "policy", args: { policy: "roulade", action: "toggle" } },
        },
      ]);
      expect(roulade.dataset.state).toBe("pending");
      click(q("policy-stand"));
      expect(hooks.sent[1].data).toEqual({
        name: "policy",
        args: { policy: "stand", action: "start" },
      });

      // The state catches up: running with progress wins over pending.
      ingest(store, "policy_state", {
        ...POLICY_STATE,
        active: "roulade",
        locked: true,
        oneshot: { name: "roulade", progress: 0.5 },
        last_error: "stand refused",
      });
      expect(roulade.dataset.state).toBe("running");
      expect(q("policy-sitstand").dataset.state).toBe("disabled");
      expect(q("chip-locked").textContent).toBe("locked");
      expect(q("chip-error").textContent).toBe("stand refused");
      expect(q("chip-variant").textContent).toBe("default");
      expect(q("chip-active").textContent).toBe("roulade");
      expect(container.querySelector('[data-testid="chip-fallen"]')).toBeNull();
    });

    it("ages out a pending click the robot never confirmed", () => {
      vi.useFakeTimers();
      ingest(store, "policy_state", POLICY_STATE);
      click(q("policy-stand"));
      expect(q("policy-stand").dataset.state).toBe("pending");
      act(() => {
        vi.advanceTimersByTime(PENDING_MS + 50);
      });
      expect(q("policy-stand").dataset.state).toBe("idle");
    });

    it("shows fallen, and leaves nav state to the map", () => {
      ingest(store, "policy_state", { ...POLICY_STATE, fallen: true });
      expect(q("chip-fallen").textContent).toBe("fallen");
      // Nav state and its cancel belong to the map panel (which is where the
      // goal marker is); a second copy in the strip read as two controls.
      ingest(store, "nav_state", {
        state: "following_path",
        goal: { x: 1.2, y: 1, yaw: 0 },
        since: 1,
        t: 2,
      });
      expect(container.querySelector('[data-testid="chip-nav"]')).toBeNull();
      expect(container.querySelector('[data-testid="chip-nav-cancel"]')).toBeNull();
    });

    it("surfaces a refused command", () => {
      hooks.result = { ok: false, reason: "not_tx" };
      click(q("control-mode-agent"));
      // The SDK's code ("not_tx") is an internal name; the strip says what
      // it means.
      expect(q("control-error").textContent).toBe("command not sent: channel is not writable");
      expect(q("control-mode-agent").getAttribute("aria-pressed")).toBe("false");
    });
  });

  describe("NavMapPanel", () => {
    const PLACES = {
      frame: "world",
      rooms: [
        { name: "kitchen", aliases: [], bounds: [0, 2, 0, 2], target: [1.2, 1, 0] },
        { name: "living", aliases: [], bounds: [-2, 0, 0, 2], target: [-1.2, 1, 3.14159] },
      ],
      objects: [{ name: "red_box", x: 1.5, y: 1.5 }],
      tagged: [{ name: "charger", x: 0.3, y: -0.4, yaw: 0 }],
      t: 1,
    };
    let calls: Record<string, unknown[][]>;
    let overlay: HTMLCanvasElement;

    beforeEach(() => {
      calls = {};
      // Every canvas gets a recording context; measureText reports a fixed width.
      vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockImplementation(() =>
        new Proxy({} as Record<string, unknown>, {
          get: (target, prop) => {
            if (typeof prop !== "string") return undefined;
            if (target[prop] === undefined) {
              target[prop] = prop === "measureText"
                ? () => ({ width: 40 })
                : (...args: unknown[]) => {
                  (calls[prop] ??= []).push(args);
                };
            }
            return target[prop];
          },
          set: (target, prop, value) => {
            if (typeof prop === "string") target[prop] = value;
            return true;
          },
        }) as unknown as CanvasRenderingContext2D
      );
      act(() => root.render(<NavMapPanel spec={NAVMAP_SPEC} store={store} teleop={hooks} />));
      overlay = q<HTMLCanvasElement>("navmap-global_costmap-overlay");
      defineSize(q("navmap-global_costmap-canvas"), 400, 300);
      defineSize(overlay, 400, 300);
      vi.spyOn(overlay, "getBoundingClientRect").mockReturnValue(
        { left: 0, top: 0, width: 400, height: 300 } as DOMRect,
      );
    });

    it("draws rooms, objects, tagged places and the path before any costmap", () => {
      expect(container.textContent).toContain("waiting for costmap...");
      ingest(store, "places", PLACES);
      expect(overlay.width).toBe(400);
      expect(overlay.height).toBe(300);
      expect(calls.strokeRect?.length).toBeGreaterThanOrEqual(2); // room bounds
      const texts = (calls.fillText ?? []).map((a) => a[0]);
      expect(texts).toEqual(
        expect.arrayContaining(["kitchen", "living", "red_box", "charger", "hub"]),
      );
      calls = {};
      ingest(store, "path", { frame: "world", points: [[0, 0], [0.5, 0.5], [1, 1]], t: 2 });
      expect(calls.moveTo?.length).toBeGreaterThanOrEqual(1);
      expect(calls.lineTo?.length).toBeGreaterThanOrEqual(2);
      expect(calls.stroke?.length).toBeGreaterThanOrEqual(1);
    });

    it("turns a click into a world goal on the goal channel", () => {
      ingest(store, "places", { ...PLACES, rooms: [] });
      // Fallback fit of (-2.2..2.2)^2 into 400x300: 68.18 px/m, origin at (200, 150).
      click(overlay, { clientX: 200, clientY: 150 });
      expect(hooks.sent).toEqual([
        { ch: "goal_request", data: { x: 0, y: 0, frame: "world" } },
      ]);
      click(overlay, { clientX: 200 + 300 / 4.4, clientY: 150 - 300 / 4.4 / 2 });
      expect(hooks.sent[1].data).toEqual({ x: 1, y: 0.5, frame: "world" });
      expect(q("navmap-note").textContent).toBe("goal → (1.00, 0.50)");
    });

    it("sends the room's target pose when its label is clicked", () => {
      ingest(store, "places", PLACES);
      // The kitchen label sits at the room centre (1, 1).
      const scale = 300 / 4.4;
      click(overlay, { clientX: 200 + scale, clientY: 150 - scale });
      expect(hooks.sent).toEqual([
        { ch: "goal_request", data: { x: 1.2, y: 1, yaw: 0, frame: "world" } },
      ]);
      expect(q("navmap-note").textContent).toBe("goal → kitchen");
    });

    it("cancels navigation from Esc on the focused map and from the chip", () => {
      ingest(store, "nav_state", {
        state: "following_path",
        goal: { x: 1, y: 1, yaw: 0 },
        since: 1,
        t: 2,
      });
      expect(q("navmap-chip").textContent).toContain("following path → (1.00, 1.00)");
      const cancel = q<HTMLButtonElement>("navmap-cancel");
      expect(mouseDownIsPrevented(cancel)).toBe(true);
      click(cancel);
      key(q("navmap-global_costmap"), "keydown", { key: "Escape" });
      expect(hooks.sent).toEqual([
        { ch: "ui_command", data: { name: "cancel_nav" } },
        { ch: "ui_command", data: { name: "cancel_nav" } },
      ]);
      // The goal pin: an arc per draw once nav_state carries a goal.
      expect(calls.arc?.length).toBeGreaterThanOrEqual(2);
    });

    it("reports a refused goal", () => {
      hooks.result = { ok: false, reason: "unknown_channel" };
      ingest(store, "places", { ...PLACES, rooms: [] });
      click(overlay, { clientX: 200, clientY: 150 });
      expect(q("navmap-note").textContent).toContain("not sent: channel missing from manifest");
    });

    it("clears the click note instead of leaving it under the map forever", () => {
      vi.useFakeTimers();
      ingest(store, "places", { ...PLACES, rooms: [] });
      click(overlay, { clientX: 200, clientY: 150 });
      expect(q("navmap-note").textContent).toContain("goal");
      act(() => {
        vi.advanceTimersByTime(NOTE_LINGER_MS - 1);
      });
      expect(container.querySelector('[data-testid="navmap-note"]')).not.toBeNull();
      act(() => {
        vi.advanceTimersByTime(2);
      });
      expect(container.querySelector('[data-testid="navmap-note"]')).toBeNull();
    });
  });
});

describe("startNavOverlay", () => {
  /** One recording context per canvas: method names in call order. */
  let seqs: string[][];
  let store: ChannelStore;
  let stop: (() => void) | null;

  beforeEach(() => {
    seqs = [];
    store = new ChannelStore();
    stop = null;
    vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockImplementation(() => {
      const seq: string[] = [];
      seqs.push(seq);
      return new Proxy({} as Record<string, unknown>, {
        get: (target, prop) => {
          if (typeof prop !== "string") return undefined;
          if (target[prop] === undefined) {
            target[prop] = prop === "measureText"
              ? () => ({ width: 40 })
              : (..._args: unknown[]) => {
                seq.push(prop);
              };
          }
          return target[prop];
        },
        set: (target, prop, value) => {
          if (typeof prop === "string") target[prop] = value;
          return true;
        },
      }) as unknown as CanvasRenderingContext2D;
    });
  });

  afterEach(() => {
    stop?.();
    vi.restoreAllMocks();
  });

  it("draws the pose on the overlay above the room labels, never on the base", async () => {
    const base = document.createElement("canvas");
    const overlay = document.createElement("canvas");
    defineSize(base, 400, 300);
    defineSize(overlay, 400, 300);
    const handle = startNavOverlay(
      store,
      {
        costmap: "global_costmap",
        pose: "odom",
        path: "path",
        places: "places",
        navState: "nav_state",
      },
      base,
      overlay,
      { lastDrawOkAtMs: 0, failures: 0 },
      { inflate: () => Promise.resolve(new Uint8Array(4)), hidden: () => false },
    );
    stop = handle.stop;
    // getContext order: the overlay, then startMapSink's display (base) and backing canvases.
    const [overlaySeq, baseSeq] = seqs;
    ingest(store, "places", {
      frame: "world",
      rooms: [{ name: "living", aliases: [], bounds: [-2, 0, 0, 2], target: [-1.2, 1, 3.14159] }],
      objects: [],
      tagged: [],
      t: 1,
    });
    ingest(store, "global_costmap", {
      bytes: new Uint8Array([1]),
      w: 2,
      h: 2,
      res: 0.5,
      origin: [0.25, -0.5, 0.0],
    });
    await act(async () => {
      await new Promise((resolve) => setTimeout(resolve, 0)); // let the inflate settle
    });
    expect(baseSeq).toContain("drawImage"); // the grid landed on the base
    expect(handle.transform()).not.toBeNull();

    overlaySeq.length = 0;
    baseSeq.length = 0;
    ingest(store, "odom", { x: -1.06, y: 0.95, z: 0.1, yaw: 2.94, ts: 3 });
    // The pose triangle (closePath + fill) is the last thing the overlay
    // draws, after the "living" label, so the robot stays visible on it.
    const label = overlaySeq.indexOf("fillText");
    const triangle = overlaySeq.lastIndexOf("closePath");
    expect(label).toBeGreaterThanOrEqual(0);
    expect(triangle).toBeGreaterThan(label);
    expect(overlaySeq[triangle + 1]).toBe("fill");
    expect(baseSeq).toEqual([]); // pose frames no longer touch the base
  });
});
