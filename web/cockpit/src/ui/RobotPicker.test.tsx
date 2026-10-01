// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import { RobotPicker } from "./RobotPicker.tsx";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

const ALPHA = { id: "alpha", name: "Alpha", model: "go2" };
const BRAVO = { id: "bravo", name: "Bravo", model: "" };

describe("RobotPicker", () => {
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

  function entries(): HTMLElement[] {
    return [...container.querySelectorAll<HTMLElement>('[data-testid^="robot-pick-"]')];
  }

  it("lists the robots sorted by name, then id, whatever the push order", () => {
    const twin = { id: "a2", name: "Alpha", model: "go2" };
    act(() =>
      root.render(<RobotPicker robots={[BRAVO, twin, ALPHA]} current={null} onPick={() => {}} />)
    );
    expect(entries().map((el) => el.dataset.testid)).toEqual([
      "robot-pick-a2",
      "robot-pick-alpha",
      "robot-pick-bravo",
    ]);
  });

  it("shows name, model and id, skipping an empty model", () => {
    act(() =>
      root.render(<RobotPicker robots={[ALPHA, BRAVO]} current={null} onPick={() => {}} />)
    );
    const [alpha, bravo] = entries();
    expect(alpha.textContent).toBe("Alphago2alpha");
    expect(bravo.textContent).toBe("Bravobravo");
  });

  it("marks the watched robot and reports a pick by id", () => {
    const picks: string[] = [];
    act(() =>
      root.render(
        <RobotPicker robots={[ALPHA, BRAVO]} current="bravo" onPick={(id) => picks.push(id)} />,
      )
    );
    const [alpha, bravo] = entries();
    expect(alpha.getAttribute("aria-current")).toBe("false");
    expect(bravo.getAttribute("aria-current")).toBe("true");
    act(() => alpha.click());
    expect(picks).toEqual(["alpha"]);
  });
});
