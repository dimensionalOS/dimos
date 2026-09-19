// @vitest-environment happy-dom
import { afterEach, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot } from "react-dom/client";
import type { Session } from "@dimos/sdk";
import { Go2Controls } from "./Go2Controls.tsx";

const data = vi.hoisted(() => ({ result: null as unknown, age: 0 }));
vi.mock("@dimos/sdk/react", () => ({
  useStatus: () => ({
    transport: { phase: "connected" },
    manifest: { channels: [{ ch: "go2_operator_command" }] },
  }),
  useChannel: (_: unknown, ch: string) => ({
    slot: {
      value: ch.endsWith("result")
        ? data.result
        : { battery: 73, light_requested: null, last_action: null },
    },
    stats: { ageMs: data.age },
  }),
}));
(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;
afterEach(() => {
  data.result = null;
  data.age = 0;
});

it("distinguishes bridge delivery from the matching robot result and hides stale battery", async () => {
  const publish = vi.fn().mockResolvedValue({});
  const session = { publish } as unknown as Session;
  const container = document.createElement("div");
  const root = createRoot(container);
  try {
    act(() => root.render(<Go2Controls session={session} />));
    expect(container.textContent).toContain("73%");
    const wave = Array.from(container.querySelectorAll("button")).find((b) =>
      b.textContent === "Wave / Shake hand"
    )!;
    await act(async () => wave.click());
    expect(publish).toHaveBeenCalledTimes(1);
    expect(publish.mock.calls[0][0]).toBe("go2_operator_command");
    const command = publish.mock.calls[0][1];
    expect(command.action).toBe("Hello");
    expect(container.textContent).toContain("waiting for robot API");
    data.result = { id: "someone-else", ok: true, message: "unrelated result" };
    act(() => root.render(<Go2Controls session={session} />));
    expect(container.textContent).not.toContain("unrelated result");
    data.result = { id: command.id, ok: false, message: "Robot API rejected" };
    data.age = 5000;
    act(() => root.render(<Go2Controls session={session} />));
    expect(container.textContent).toContain("Robot API rejected");
    expect(container.textContent).not.toContain("73%");
    expect(publish).toHaveBeenCalledTimes(1);
  } finally {
    act(() => root.unmount());
  }
});
