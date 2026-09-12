// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { PanelSpec } from "@dimos/shared";
import type { Manifest } from "@dimos/shared/manifest";
import { ChannelStore } from "@dimos/sdk";
import { pageTabs, PageView } from "./PageView.tsx";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

function panel(id: string, title = ""): PanelSpec {
  // voxels is an unknown kind in this build: pages render via UnknownPanel.
  return { id, kind: "voxels", title, channels: [], params: {} };
}

function manifest(pages: string[], panels: PanelSpec[]): Manifest {
  return { version: 1, channels: [], panels, layout: null, pages };
}

describe("pageTabs", () => {
  it("lists the pages in manifest order, labelled title || id", () => {
    const m = manifest(["st", "cam"], [panel("cam", "Front camera"), panel("st")]);
    expect(pageTabs(m)).toEqual([
      { id: "st", label: "st" },
      { id: "cam", label: "Front camera" },
    ]);
  });
});

describe("PageView", () => {
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

  function render(m: Manifest, page: string | null) {
    act(() =>
      root.render(
        <PageView manifest={m} page={page} store={store}>
          <span data-testid="grid-content">the grid</span>
        </PageView>,
      )
    );
  }

  const grid = () => container.querySelector('[data-testid="grid-content"]');
  const st = () => container.querySelector('[data-testid="panel-st"]');

  it("shows the grid for overview and only the page for a page id", () => {
    const m = manifest(["st"], [panel("st", "Statistici")]);
    render(m, null);
    expect(grid()).not.toBeNull();
    expect(st()).toBeNull();
    // The tab strip lives in the header; nothing here is a tab.
    expect(container.querySelector('[role="tab"]')).toBeNull();

    render(m, "st");
    // The grid unmounts: a hidden canvas would keep decoding.
    expect(grid()).toBeNull();
    expect(st()).not.toBeNull();
    expect(container.textContent).toContain("unknown panel kind voxels");

    render(m, null);
    expect(grid()).not.toBeNull();
    expect(st()).toBeNull();
  });

  it("falls back to the grid for an id the manifest lacks", () => {
    render(manifest([], []), "gone");
    expect(grid()).not.toBeNull();
  });

  it("renders a page whose panel id is overview", () => {
    render(manifest(["overview"], [panel("overview")]), "overview");
    expect(grid()).toBeNull();
    expect(container.querySelector('[data-testid="panel-overview"]')).not.toBeNull();
  });
});
