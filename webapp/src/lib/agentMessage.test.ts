import { describe, it, expect } from "vitest";
import { classifyAgentMessage } from "./agentMessage";

describe("classifyAgentMessage", () => {
  it("drops empty / whitespace frames", () => {
    expect(classifyAgentMessage("")).toBeNull();
    expect(classifyAgentMessage("   ")).toBeNull();
  });

  it("parses an ai envelope (the spoken reply)", () => {
    expect(
      classifyAgentMessage('{"kind":"ai","text":"I moved forward."}'),
    ).toEqual({ kind: "ai", text: "I moved forward." });
  });

  it("parses a tool envelope (status — not spoken)", () => {
    expect(
      classifyAgentMessage('{"kind":"tool","text":"Navigation goal reached"}'),
    ).toEqual({ kind: "tool", text: "Navigation goal reached" });
  });

  it("parses a system envelope", () => {
    expect(
      classifyAgentMessage('{"kind":"system","text":"reconnected"}'),
    ).toEqual({ kind: "system", text: "reconnected" });
  });

  it("defaults missing or unknown kind to ai", () => {
    expect(classifyAgentMessage('{"text":"hi there"}')).toEqual({
      kind: "ai",
      text: "hi there",
    });
    expect(classifyAgentMessage('{"kind":"weird","text":"hey"}')).toEqual({
      kind: "ai",
      text: "hey",
    });
  });

  it("drops envelopes with empty text", () => {
    expect(classifyAgentMessage('{"kind":"tool","text":""}')).toBeNull();
  });

  it("treats legacy plain text as an ai reply", () => {
    expect(classifyAgentMessage("Hello! I am Goldie.")).toEqual({
      kind: "ai",
      text: "Hello! I am Goldie.",
    });
  });

  it("strips wrapping quotes on legacy plain text", () => {
    expect(classifyAgentMessage("'Hello there'")!.text).toBe("Hello there");
  });
});
