// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { PanelSpec } from "@dimos/shared";
import { PanelFrame } from "./PanelFrame.tsx";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

function spec(over: Partial<PanelSpec> = {}): PanelSpec {
  return { id: "cam", kind: "video", title: "", channels: [], params: {}, ...over };
}

describe("PanelFrame", () => {
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

  const frame = () => container.querySelector('[data-testid="panel-cam"]')!;
  const maxButton = () =>
    container.querySelector<HTMLButtonElement>('[data-testid="panel-cam-max"]')!;

  function render(s: PanelSpec) {
    act(() => root.render(<PanelFrame spec={s}>body</PanelFrame>));
  }

  it("shows the title, falling back to the panel id when untitled", () => {
    render(spec());
    expect(frame().textContent).toContain("cam");
    render(spec({ title: "Front camera" }));
    expect(frame().textContent).toContain("Front camera");
  });

  it("renders the badge slot next to the maximize button", () => {
    act(() =>
      root.render(
        <PanelFrame spec={spec()} badge={<span data-testid="badge-slot">9.9 fps</span>}>
          body
        </PanelFrame>,
      )
    );
    expect(container.querySelector('[data-testid="badge-slot"]')!.textContent).toBe("9.9 fps");
  });

  // happy-dom has no layout engine, so maximize asserts state (attribute,
  // aria-label), never geometry; the overlay CSS is checked in the live demo.
  it("maximizes on click and restores on Esc", () => {
    render(spec());
    expect(frame().getAttribute("data-maximized")).toBeNull();
    expect(maxButton().getAttribute("aria-label")).toBe("maximize");

    act(() => maxButton().click());
    expect(frame().getAttribute("data-maximized")).toBe("true");
    expect(maxButton().getAttribute("aria-label")).toBe("restore");

    // A non-Esc key leaves it maximized.
    act(() => {
      document.dispatchEvent(new KeyboardEvent("keydown", { key: "a", bubbles: true }));
    });
    expect(frame().getAttribute("data-maximized")).toBe("true");

    act(() => {
      document.dispatchEvent(new KeyboardEvent("keydown", { key: "Escape", bubbles: true }));
    });
    expect(frame().getAttribute("data-maximized")).toBeNull();
  });

  it("restores on a second click too", () => {
    render(spec());
    act(() => maxButton().click());
    act(() => maxButton().click());
    expect(frame().getAttribute("data-maximized")).toBeNull();
  });
});
