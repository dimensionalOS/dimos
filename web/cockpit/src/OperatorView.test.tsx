// @vitest-environment happy-dom
import { expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot } from "react-dom/client";
import { ChannelStore } from "@dimos/sdk";
import { OperatorView } from "./OperatorView.tsx";

vi.mock(
  "./panels/VideoPanel.tsx",
  () => ({ VideoPanel: () => <canvas data-testid="video-sink" /> }),
);
vi.mock("./panels/MapPanel.tsx", () => ({ MapPanel: () => <canvas data-testid="map-sink" /> }));
(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

it("swaps stage placement without replacing either live renderer", () => {
  const container = document.createElement("div");
  const root = createRoot(container);
  try {
    act(() =>
      root.render(
        <OperatorView
          store={new ChannelStore()}
          teleop={undefined}
          panels={[
            { id: "camera", kind: "video", title: "Camera", channels: ["color_image"], params: {} },
            {
              id: "map",
              kind: "map2d",
              title: "Map",
              channels: ["global_costmap", "odom"],
              params: {},
            },
          ]}
        />,
      )
    );
    const video = container.querySelector('[data-testid="video-sink"]');
    const map = container.querySelector('[data-testid="map-sink"]');
    const cameraStage = container.querySelector('[data-testid="operator-camera"]')!;
    const originalClass = cameraStage.className;
    const buttons = container.querySelectorAll("nav button");
    act(() => (buttons[1] as HTMLButtonElement).click());
    expect(buttons[1].getAttribute("aria-pressed")).toBe("true");
    expect(cameraStage.className).not.toBe(originalClass);
    expect(container.querySelector('[data-testid="video-sink"]')).toBe(video);
    expect(container.querySelector('[data-testid="map-sink"]')).toBe(map);
    expect(container.textContent).toContain("No teleop panel advertised");
  } finally {
    act(() => root.unmount());
  }
});
