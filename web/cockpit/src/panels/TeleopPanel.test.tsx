// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { Msg, PanelSpec } from "@dimos/shared";
import type { Manifest } from "@dimos/shared/manifest";
import { ChannelStore, StatusStore } from "@dimos/sdk";
import type { TeleopHooks, TxResult } from "@dimos/sdk/internal/teleop";
import { TeleopPanel } from "./TeleopPanel.tsx";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

const CH = "tele_cmd_vel";
const SPEC: PanelSpec = { id: "p0", kind: "teleop", title: "", channels: [CH], params: {} };
const MANIFEST: Manifest = {
  version: 1,
  channels: [
    {
      ch: CH,
      dir: "tx",
      encoding: "twist.json.v1",
      delivery: "latest",
      maxHz: 15,
      params: { maxLinear: 0.8, maxAngular: 1.0, boost: 2.0, watchdogMs: 300 },
    },
  ],
  panels: [SPEC],
  layout: "p0",
  pages: [],
};

class FakeHooks implements TeleopHooks {
  controls: Msg[] = [];
  datagrams: Msg[] = [];
  cbs = new Set<(msg: Msg) => void>();
  status = new StatusStore();

  control = (msg: Msg) => this.controls.push(msg);
  datagram = (msg: Msg) => this.datagrams.push(msg);
  onMsg = (cb: (msg: Msg) => void) => {
    this.cbs.add(cb);
    return () => this.cbs.delete(cb);
  };

  tx(): TxResult {
    return { ok: false, reason: "disconnected" };
  }

  reply(msg: Msg): void {
    for (const cb of this.cbs) cb(msg);
  }
}

describe("TeleopPanel", () => {
  let container: HTMLElement;
  let root: Root;
  let hooks: FakeHooks;

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    hooks = new FakeHooks();
    hooks.status.update({ transport: { phase: "connected" }, manifest: MANIFEST });
    act(() => root.render(<TeleopPanel spec={SPEC} store={new ChannelStore()} teleop={hooks} />));
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
  });

  function pad(): HTMLElement {
    const el = container.querySelector<HTMLElement>(`[data-testid="teleop-${CH}"]`);
    if (el === null) throw new Error("teleop pad not rendered");
    return el;
  }

  function armPanel(): void {
    act(() => pad().focus());
    act(() => hooks.reply({ t: "teleop_started" }));
  }

  function key(type: "keydown" | "keyup", code: string): void {
    act(() => {
      pad().dispatchEvent(new KeyboardEvent(type, { code, bubbles: true, cancelable: true }));
    });
  }

  it("arms on focus only after the relay grants the lease", () => {
    expect(pad().dataset.state).toBe("disarmed");
    expect(container.textContent).toContain("click to arm");
    act(() => pad().focus());
    expect(hooks.controls).toEqual([{ t: "teleop_start" }]);
    expect(pad().dataset.state).toBe("arming");
    act(() => hooks.reply({ t: "teleop_started" }));
    expect(pad().dataset.state).toBe("armed");
  });

  it("shows the held reason when the lease is refused", () => {
    act(() => pad().focus());
    act(() => hooks.reply({ t: "error", code: "teleop_held", message: "held" }));
    expect(pad().dataset.state).toBe("disarmed");
    expect(container.textContent).toContain("teleop held by another viewer");
  });

  it("re-arms by click while still focused after a lease refusal", () => {
    act(() => pad().focus());
    act(() => hooks.reply({ t: "error", code: "teleop_held", message: "held" }));
    expect(pad().dataset.state).toBe("disarmed");
    // The pad still holds focus, so no further focus event will fire; the
    // click alone must request the lease again.
    act(() => pad().click());
    expect(hooks.controls).toEqual([{ t: "teleop_start" }, { t: "teleop_start" }]);
    // While arming, further clicks are no-ops (no duplicate teleop_start).
    act(() => pad().click());
    expect(hooks.controls).toHaveLength(2);
    act(() => hooks.reply({ t: "teleop_started" }));
    expect(pad().dataset.state).toBe("armed");
  });

  it("re-arms by click on the still-focused pad after a reconnect", () => {
    armPanel();
    act(() => {
      hooks.status.update({
        transport: { phase: "reconnecting", attempt: 1, retryAtMs: 0 },
      });
    });
    expect(pad().dataset.state).toBe("stopped");
    act(() => hooks.status.update({ transport: { phase: "connected" } }));
    expect(pad().dataset.state).toBe("disarmed");
    expect(container.textContent).toContain("connection lost");
    act(() => pad().click());
    expect(hooks.controls.at(-1)).toEqual({ t: "teleop_start" });
    act(() => hooks.reply({ t: "teleop_started" }));
    expect(pad().dataset.state).toBe("armed");
  });

  it("drives from key events and reflects them in cluster and readout", () => {
    armPanel();
    key("keydown", "KeyW");
    expect(hooks.datagrams.at(-1)).toMatchObject({ t: "twist", vx: 0.8, vy: 0, wz: 0 });
    const wKey = container.querySelector('[data-testid="teleop-key-W"]');
    expect(wKey?.getAttribute("data-pressed")).toBe("true");
    expect(container.textContent).toContain("vx 0.80");
    key("keyup", "KeyW");
    expect(hooks.datagrams.at(-1)).toMatchObject({ t: "twist", vx: 0, vy: 0, wz: 0 });
  });

  it("Space e-stops on both paths and stays armed", () => {
    armPanel();
    key("keydown", "KeyW");
    key("keydown", "Space");
    expect(hooks.controls.at(-1)?.t).toBe("stop");
    expect(hooks.datagrams.at(-1)?.t).toBe("stop");
    expect(pad().dataset.state).toBe("armed");
  });

  it("Escape disarms and releases the lease", () => {
    armPanel();
    key("keydown", "Escape");
    expect(pad().dataset.state).toBe("disarmed");
    expect(hooks.controls.at(-1)).toEqual({ t: "teleop_stop" });
  });

  it("focus loss and window blur disarm", () => {
    armPanel();
    act(() => pad().blur());
    expect(pad().dataset.state).toBe("disarmed");

    armPanel();
    act(() => {
      globalThis.dispatchEvent(new Event("blur"));
    });
    expect(pad().dataset.state).toBe("disarmed");
  });

  it("unmount releases the lease", () => {
    armPanel();
    act(() => root.unmount());
    expect(hooks.controls.at(-1)).toEqual({ t: "teleop_stop" });
  });

  it("shows STOPPED while the transport is down and refuses to arm", () => {
    act(() => {
      hooks.status.update({
        transport: { phase: "reconnecting", attempt: 1, retryAtMs: 0 },
      });
    });
    expect(pad().dataset.state).toBe("stopped");
    expect(container.textContent).toContain("connection lost");
    act(() => pad().focus());
    expect(hooks.controls).toHaveLength(0);
  });
});
