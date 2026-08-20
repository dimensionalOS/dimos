// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { SessionStatus } from "@dimos/sdk";
import { StatusBar } from "./StatusBar.tsx";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

function makeStatus(over: Partial<SessionStatus> = {}): SessionStatus {
  return {
    transport: { phase: "connected" },
    robots: [],
    watchedRobot: null,
    manifest: null,
    manifestUnsupported: false,
    epoch: 0,
    lastError: null,
    ...over,
  };
}

describe("StatusBar", () => {
  let container: HTMLElement;
  let root: Root;

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
  });

  function render(status: SessionStatus) {
    act(() => root.render(<StatusBar status={status} />));
  }

  function testId(id: string): Element {
    return container.querySelector(`[data-testid="${id}"]`)!;
  }

  it("shows the transport phase and the picked robot", () => {
    render(makeStatus({ watchedRobot: { id: "a", name: "Go2", model: "go2" } }));
    expect(testId("status").getAttribute("data-phase")).toBe("connected");
    expect(testId("status").textContent).toBe("connected");
    expect(testId("robot").textContent).toBe("Go2 (go2)");
  });

  it("shows 'no robot' when none is picked", () => {
    render(makeStatus());
    expect(testId("robot").textContent).toBe("no robot");
  });

  it("shows the retry countdown and close reason while reconnecting", () => {
    render(
      makeStatus({
        transport: {
          phase: "reconnecting",
          attempt: 3,
          retryAtMs: Date.now() + 2000,
          reason: "kicked",
        },
      }),
    );
    expect(testId("status").getAttribute("data-phase")).toBe("reconnecting");
    expect(container.textContent).toContain("attempt 3");
    expect(container.textContent).toContain("kicked");
  });

  it("shows lastError while set and drops it once cleared", () => {
    render(makeStatus({
      lastError: { code: "relay_error", message: "unknown_robot: no robot a" },
    }));
    expect(container.textContent).toContain("unknown_robot: no robot a");
    render(makeStatus({ lastError: null }));
    expect(container.textContent).not.toContain("unknown_robot");
  });
});
