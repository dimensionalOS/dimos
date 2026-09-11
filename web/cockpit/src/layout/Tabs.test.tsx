// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { PanelSpec } from "@dimos/shared";
import type { Manifest } from "@dimos/shared/manifest";
import { ChannelStore } from "@dimos/sdk";
import { Tabs } from "./Tabs.tsx";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

function panel(id: string, title = ""): PanelSpec {
  // voxels is an unknown kind in this build: pages render via UnknownPanel.
  return { id, kind: "voxels", title, channels: [], params: {} };
}

function manifest(pages: string[], panels: PanelSpec[]): Manifest {
  return { version: 1, channels: [], panels, layout: null, pages };
}

describe("Tabs", () => {
  let container: HTMLElement;
  let root: Root;
  let store: ChannelStore;

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    store = new ChannelStore();
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
  });

  function render(m: Manifest) {
    act(() =>
      root.render(
        <Tabs manifest={m} store={store}>
          <span data-testid="grid-content">the grid</span>
        </Tabs>,
      )
    );
  }

  const tab = (id: string) =>
    container.querySelector<HTMLButtonElement>(`[data-testid="tab-${id}"]`);

  it("renders no tab bar while pages are empty (all of T7)", () => {
    render(manifest([], []));
    expect(container.querySelector('[role="tablist"]')).toBeNull();
    expect(container.querySelector('[data-testid="grid-content"]')).not.toBeNull();
  });

  it("switches between the grid and a page, mounting only the active one", () => {
    render(manifest(["st"], [panel("st", "Statistici")]));
    expect(tab("overview")!.getAttribute("aria-selected")).toBe("true");
    expect(tab("st")!.textContent).toBe("Statistici"); // label = title || id
    expect(container.querySelector('[data-testid="grid-content"]')).not.toBeNull();
    expect(container.querySelector('[data-testid="panel-st"]')).toBeNull();

    act(() => tab("st")!.click());
    expect(tab("st")!.getAttribute("aria-selected")).toBe("true");
    // The grid unmounts: a hidden canvas would keep decoding.
    expect(container.querySelector('[data-testid="grid-content"]')).toBeNull();
    expect(container.querySelector('[data-testid="panel-st"]')).not.toBeNull();
    expect(container.textContent).toContain("unknown panel kind voxels");

    act(() => tab("overview")!.click());
    expect(container.querySelector('[data-testid="grid-content"]')).not.toBeNull();
    expect(container.querySelector('[data-testid="panel-st"]')).toBeNull();
  });

  it("labels an untitled page tab with its panel id", () => {
    render(manifest(["st"], [panel("st")]));
    expect(tab("st")!.textContent).toBe("st");
  });
});
