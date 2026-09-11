// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { SessionStatus } from "@dimos/sdk";
import type { PageTab } from "../layout/PageView.tsx";
import { StatusBar, type View } from "./StatusBar.tsx";

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

  type Props = Parameters<typeof StatusBar>[0];

  function render(status: SessionStatus, over: Partial<Props> = {}) {
    act(() =>
      root.render(
        <StatusBar
          status={status}
          view="panels"
          onViewChange={() => {}}
          pages={[]}
          page={null}
          onPageChange={() => {}}
          {...over}
        />,
      )
    );
  }

  function testId(id: string): Element {
    return container.querySelector(`[data-testid="${id}"]`)!;
  }

  const STATS: PageTab[] = [{ id: "st", label: "Stats" }];

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

  it("offers the panels and channels views and reports the pick", () => {
    const picks: View[] = [];
    render(makeStatus(), { onViewChange: (v) => picks.push(v) });
    expect(testId("view-panels").getAttribute("aria-selected")).toBe("true");
    expect(testId("view-channels").getAttribute("aria-selected")).toBe("false");
    act(() => (testId("view-channels") as HTMLElement).click());
    expect(picks).toEqual(["channels"]);
  });

  it("renders Overview plus the page tabs, marks the open one and reports the pick", () => {
    const picks: (string | null)[] = [];
    render(makeStatus(), { pages: STATS, page: "st", onPageChange: (id) => picks.push(id) });
    expect(testId("tab-overview").textContent).toBe("Overview");
    expect(testId("tab-overview").getAttribute("aria-selected")).toBe("false");
    expect(testId("tab-page-st").textContent).toBe("Stats");
    expect(testId("tab-page-st").getAttribute("aria-selected")).toBe("true");
    act(() => (testId("tab-overview") as HTMLElement).click());
    expect(picks).toEqual([null]);
  });

  it("renders no page tabs without pages, and marks none open in the channels view", () => {
    render(makeStatus());
    expect(container.querySelector('[data-testid="tab-overview"]')).toBeNull();
    render(makeStatus(), { pages: STATS, page: "st", view: "channels" });
    expect(testId("tab-page-st").getAttribute("aria-selected")).toBe("false");
    expect(testId("tab-overview").getAttribute("aria-selected")).toBe("false");
  });

  it("does not collide with a manifest page named overview", () => {
    render(makeStatus(), { pages: [{ id: "overview", label: "Page" }], page: "overview" });
    expect(container.querySelectorAll('[data-testid="tab-overview"]')).toHaveLength(1);
    expect(testId("tab-overview").getAttribute("aria-selected")).toBe("false");
    expect(testId("tab-page-overview").getAttribute("aria-selected")).toBe("true");
  });
});
