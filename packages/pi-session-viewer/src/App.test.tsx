import { render, screen, within } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { describe, expect, it } from "vitest";

import { App } from "./App";
import type { PiSessionViewModel } from "./types";

const session: PiSessionViewModel = {
  schemaVersion: "1.0",
  summary: {
    id: "session-1",
    status: "partial",
    startedAt: "2026-07-23T12:00:00Z",
    model: "gpt-test",
    provider: "openai",
    entryCount: 4,
    branchCount: 1,
    totalUsage: { input: 10, output: 5, totalTokens: 15 },
  },
  rootIds: ["user"],
  headIds: ["tool-result", "alternate"],
  defaultHeadId: "tool-result",
  entries: [
    {
      id: "user",
      parentId: null,
      depth: 0,
      type: "message",
      role: "user",
      timestamp: "2026-07-23T12:00:01Z",
      timestampMs: 1_753_272_001_000,
      title: "User",
      preview: "Inspect the room",
      parts: [{ type: "text", text: "Inspect the room" }],
      raw: { type: "message", id: "user" },
    },
    {
      id: "assistant",
      parentId: "user",
      depth: 1,
      type: "message",
      role: "assistant",
      timestamp: "2026-07-23T12:00:02Z",
      title: "Assistant",
      preview: "Found it",
      model: "gpt-test",
      stopReason: "toolUse",
      usage: { totalTokens: 15 },
      parts: [
        { type: "thinking", text: "Check the map", redacted: false },
        {
          type: "text",
          text: "Found it. [remote](https://example.com) ![pixel](https://example.com/pixel.png)",
        },
        {
          type: "tool_call",
          callId: "call-1",
          name: "locate",
          arguments: { query: "mug" },
        },
        { type: "image", mimeType: "image/png", omitted: true },
      ],
      raw: { type: "message", id: "assistant" },
    },
    {
      id: "tool-result",
      parentId: "assistant",
      depth: 2,
      type: "message",
      role: "tool",
      timestamp: "2026-07-23T12:00:03Z",
      title: "Tool · locate",
      preview: "mug at (1, 2)",
      parts: [
        {
          type: "tool_result",
          callId: "call-1",
          name: "locate",
          content: "mug at (1, 2)",
          isError: false,
          details: { location: [1, 2] },
        },
      ],
      raw: { type: "message", id: "tool-result" },
    },
    {
      id: "alternate",
      parentId: "user",
      depth: 1,
      type: "model_change",
      role: "state",
      timestamp: "2026-07-23T12:00:04Z",
      title: "model change",
      preview: "",
      parts: [
        {
          type: "state",
          label: "model change",
          value: '{"modelId":"gpt-next"}',
        },
      ],
      raw: { type: "model_change", id: "alternate" },
    },
  ],
};

describe("App", () => {
  it("renders the read-only native session contract without mutation controls", async () => {
    const user = userEvent.setup();
    const { container } = render(<App session={session} />);

    expect(screen.getByText("Read only")).toBeInTheDocument();
    expect(screen.getByText("partial")).toBeInTheDocument();
    expect(screen.getAllByText("Inspect the room")).toHaveLength(2);
    expect(screen.getByText("Reasoning")).toBeInTheDocument();
    const transcript = container.querySelector("#transcript");
    expect(transcript).not.toBeNull();
    const transcriptView = within(transcript as HTMLElement);
    expect(transcriptView.getByText("locate")).toBeInTheDocument();
    expect(transcriptView.getByText("Succeeded")).toBeInTheDocument();
    expect(transcriptView.getByText("Input")).toBeInTheDocument();
    expect(transcriptView.getByText("Output")).toBeInTheDocument();
    expect(transcriptView.getByText("mug at (1, 2)")).toBeInTheDocument();
    expect(transcriptView.getByText(/"location":/)).toBeInTheDocument();
    expect(transcriptView.getByText("2 conversation items")).toBeInTheDocument();
    expect(transcriptView.getByText("3 native events")).toBeInTheDocument();
    expect(
      transcriptView.queryByRole("button", { name: "Inspect Tool · locate entry" }),
    ).not.toBeInTheDocument();
    expect(screen.getByText("15 tokens")).toBeInTheDocument();
    expect(
      screen.getByText("Embedded image omitted from review (image/png)"),
    ).toBeInTheDocument();
    expect(container.querySelector("textarea")).not.toBeInTheDocument();
    expect(container.querySelector("form")).not.toBeInTheDocument();
    expect(screen.queryByRole("button", { name: /send|submit|retry/i })).toBeNull();
    expect(container.querySelector('a[href^="http"]')).not.toBeInTheDocument();
    expect(container.querySelector('img[src^="http"]')).not.toBeInTheDocument();

    await user.click(
      screen.getByRole("button", { name: "Inspect Assistant entry" }),
    );
    const details = screen.getByRole("complementary", {
      name: "Entry details",
    });
    expect(within(details).getByText("assistant")).toBeInTheDocument();
    expect(within(details).getByText(/"totalTokens": 15/)).toBeInTheDocument();
  });

  it("changes only the selected ancestry when a branch is chosen", async () => {
    const user = userEvent.setup();
    render(<App session={session} />);

    await user.click(screen.getByRole("button", { name: /model change/i }));

    expect(screen.getByText('{"modelId":"gpt-next"}')).toBeInTheDocument();
    expect(screen.queryByText("Found it.")).not.toBeInTheDocument();
    expect(screen.getByText("2 conversation items")).toBeInTheDocument();
    expect(screen.getByText("2 native events")).toBeInTheDocument();
  });

  it("promotes sandbox commands instead of repeating the tool name", () => {
    const sandboxSession = structuredClone(session);
    const call = sandboxSession.entries[1].parts[2];
    const result = sandboxSession.entries[2].parts[0];
    if (call.type !== "tool_call" || result.type !== "tool_result") {
      throw new Error("Invalid test fixture");
    }
    call.name = "sandbox_exec";
    call.arguments = {
      command: "find /input -maxdepth 2 -type f -print",
      timeout: 30,
    };
    result.name = "sandbox_exec";
    result.content = JSON.stringify({
      stdout: "first line\nsecond line\n",
      stderr: "",
      exit_code: 0,
    });
    result.details = {};
    sandboxSession.entries[2].title = "Tool · sandbox_exec";

    const { container } = render(<App session={sandboxSession} />);
    const transcript = container.querySelector("#transcript");
    expect(transcript).not.toBeNull();
    const transcriptView = within(transcript as HTMLElement);

    expect(
      transcriptView.getByText("find /input -maxdepth 2 -type f -print"),
    ).toBeInTheDocument();
    expect(transcriptView.queryByText("sandbox_exec")).not.toBeInTheDocument();
    expect(transcriptView.queryByText("Input")).not.toBeInTheDocument();
    expect(transcriptView.getByText("Options")).toBeInTheDocument();
    expect(transcriptView.getByText(/"timeout": 30/)).toBeInTheDocument();
    expect(
      transcriptView.queryByText("Details", { selector: ".tool-section-label" }),
    ).not.toBeInTheDocument();
    expect(transcriptView.getByText("Output")).toBeInTheDocument();

    const tree = screen.getByRole("navigation", { name: "Session branches" });
    expect(
      within(tree).getByText("find /input -maxdepth 2 -type f -print"),
    ).toBeInTheDocument();
    expect(within(tree).getByText("Succeeded")).toBeInTheDocument();
    expect(within(tree).queryByText("Tool · sandbox_exec")).not.toBeInTheDocument();
    expect(transcriptView.getByText("Terminal output")).toBeInTheDocument();
    expect(transcriptView.getByText("exit 0")).toBeInTheDocument();
    expect(transcript?.querySelector(".terminal-stream")).toHaveTextContent(
      /first line\s+second line/,
    );
    expect(transcriptView.queryByText(/"stdout"/)).not.toBeInTheDocument();
  });
});
