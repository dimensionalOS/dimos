import { describe, expect, it } from "vitest";

import { coalesceToolExchanges, parseSandboxOutput } from "./session";
import type { SessionEntry } from "./types";

function entry(
  id: string,
  role: SessionEntry["role"],
  parts: SessionEntry["parts"],
): SessionEntry {
  return {
    id,
    parentId: null,
    depth: 0,
    type: "message",
    role,
    timestamp: "2026-07-23T12:00:00Z",
    title: id,
    preview: "",
    parts,
    raw: { id },
  };
}

describe("coalesceToolExchanges", () => {
  it("groups a later result with its call without mutating native entries", () => {
    const entries = [
      entry("assistant", "assistant", [
        {
          type: "tool_call",
          callId: "call-1",
          name: "locate",
          arguments: { query: "mug" },
        },
      ]),
      entry("result", "tool", [
        {
          type: "tool_result",
          callId: "call-1",
          name: "locate",
          content: "found",
          isError: false,
        },
      ]),
    ];

    const conversation = coalesceToolExchanges(entries);

    expect(conversation).toHaveLength(1);
    expect(conversation[0].parts.map((part) => part.type)).toEqual([
      "tool_call",
      "tool_result",
    ]);
    expect(entries[0].parts).toHaveLength(1);
    expect(entries[1].parts).toHaveLength(1);
  });

  it("keeps unmatched results and calls visible", () => {
    const entries = [
      entry("call", "assistant", [
        {
          type: "tool_call",
          callId: "missing-result",
          name: "move",
          arguments: {},
        },
      ]),
      entry("result", "tool", [
        {
          type: "tool_result",
          callId: "missing-call",
          name: "look",
          content: "visible",
          isError: true,
        },
      ]),
    ];

    expect(coalesceToolExchanges(entries)).toHaveLength(2);
  });
});

describe("parseSandboxOutput", () => {
  it("decodes terminal streams without exposing the JSON envelope", () => {
    expect(
      parseSandboxOutput(
        JSON.stringify({
          stdout: "hello\nworld\n",
          stderr: "warning\n",
          exit_code: 2,
        }),
      ),
    ).toEqual({
      stdout: "hello\nworld\n",
      stderr: "warning\n",
      exitCode: 2,
    });
  });

  it("rejects malformed and unrelated tool output", () => {
    expect(parseSandboxOutput("plain text")).toBeNull();
    expect(parseSandboxOutput('{"stdout":"missing stderr"}')).toBeNull();
  });
});
