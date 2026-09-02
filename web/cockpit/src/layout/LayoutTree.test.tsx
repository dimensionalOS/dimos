// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { ChannelSpec, PanelSpec } from "@dimos/shared";
import type { Manifest } from "@dimos/shared/manifest";
import { ChannelStore } from "@dimos/sdk";
import { LayoutTree } from "./LayoutTree.tsx";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

function spec(ch: string): ChannelSpec {
  return { ch, dir: "rx", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20, params: {} };
}

function panel(id: string, kind: string, channels: string[] = []): PanelSpec {
  return { id, kind, title: "", channels, params: {} };
}

function manifest(over: Partial<Manifest>): Manifest {
  return { version: 1, channels: [], panels: [], layout: null, pages: [], ...over };
}

// readout is an unknown kind in this build: panels render via UnknownPanel,
// which needs no canvas mocking.
const CAM = panel("cam", "readout", ["odom"]);
const POSE = panel("pose", "readout", ["odom"]);
const AUX = panel("aux", "readout", ["odom"]);

describe("LayoutTree", () => {
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
    act(() => root.render(<LayoutTree manifest={m} store={store} />));
  }

  function cells(): HTMLElement[] {
    return [...container.querySelectorAll<HTMLElement>("div[style]")].filter(
      (el) => el.style.flexGrow !== "",
    );
  }

  it("applies shares as flex-grow, defaulting absent shares to 1", () => {
    render(
      manifest({
        channels: [spec("odom")],
        panels: [CAM, POSE],
        layout: { row: ["cam", "pose"], shares: [2.5, 1.5] },
      }),
    );
    expect(cells().map((el) => el.style.flexGrow)).toEqual(["2.5", "1.5"]);

    render(
      manifest({ channels: [spec("odom")], panels: [CAM, POSE], layout: { row: ["cam", "pose"] } }),
    );
    expect(cells().map((el) => el.style.flexGrow)).toEqual(["1", "1"]);
  });

  it("sizes strip kinds to content: no share, the rest goes to the other cells", () => {
    const bar = panel("bar", "control", ["mode", "policy_state", "nav_state", "ui_command"]);
    render(
      manifest({
        channels: [spec("odom")],
        panels: [bar, CAM],
        layout: { col: ["bar", "cam"], shares: [1, 11] },
      }),
    );
    expect(container.querySelector('[data-testid="panel-bar"]')).not.toBeNull();
    // Only the readout cell carries a flex-grow; the strip cell has none.
    expect(cells().map((el) => el.style.flexGrow)).toEqual(["11"]);
  });

  it("renders nested rows/cols and every referenced panel", () => {
    render(
      manifest({
        channels: [spec("odom")],
        panels: [CAM, POSE, AUX],
        layout: { col: [{ row: ["cam", "pose"], shares: [1.5, 2.5] }, "aux"] },
      }),
    );
    for (const id of ["cam", "pose", "aux"]) {
      expect(container.querySelector(`[data-testid="panel-${id}"]`)).not.toBeNull();
    }
    // Outer col: 2 cells; inner row: 2 cells.
    expect(cells().map((el) => el.style.flexGrow)).toEqual(["1", "1.5", "2.5", "1"]);
  });

  it("renders a single-panel layout (string node)", () => {
    render(manifest({ channels: [spec("odom")], panels: [CAM], layout: "cam" }));
    expect(container.querySelector('[data-testid="panel-cam"]')).not.toBeNull();
  });

  it("falls back to one row of all non-page panels when layout is null", () => {
    render(manifest({ channels: [spec("odom")], panels: [CAM, POSE, AUX], pages: ["aux"] }));
    expect(container.querySelector('[data-testid="panel-cam"]')).not.toBeNull();
    expect(container.querySelector('[data-testid="panel-pose"]')).not.toBeNull();
    // A page panel renders in its tab, never in the grid fallback.
    expect(container.querySelector('[data-testid="panel-aux"]')).toBeNull();
  });

  it("renders nothing for a panel-less manifest", () => {
    render(manifest({ channels: [spec("odom")] }));
    expect(container.innerHTML).toBe("");
  });

  it("renders unknown kinds through UnknownPanel and leaves unreferenced panels out", () => {
    render(manifest({ channels: [spec("odom")], panels: [CAM, POSE], layout: "cam" }));
    expect(container.textContent).toContain("unknown panel kind readout");
    expect(container.querySelector('[data-testid="panel-pose"]')).toBeNull();
  });

  it("renders known kinds through the registry", () => {
    vi.stubGlobal("createImageBitmap", () => Promise.reject(new Error("unused")));
    const video = panel("cam", "video", ["color_image"]);
    render(
      manifest({
        channels: [
          {
            ch: "color_image",
            dir: "rx",
            encoding: "jpeg.v1",
            delivery: "latest",
            maxHz: 15,
            params: {},
          },
        ],
        panels: [video],
        layout: "cam",
      }),
    );
    expect(container.querySelector('[data-testid="video-color_image-canvas"]')).not.toBeNull();
    vi.unstubAllGlobals();
  });
});
