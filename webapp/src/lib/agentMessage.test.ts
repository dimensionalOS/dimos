import { describe, it, expect } from "vitest";
import { classifyAgentMessage } from "./agentMessage";

describe("classifyAgentMessage", () => {
  it("drops empty / whitespace frames", () => {
    expect(classifyAgentMessage("")).toBeNull();
    expect(classifyAgentMessage("   ")).toBeNull();
  });

  it("classifies a spoken line and strips the prefix", () => {
    expect(classifyAgentMessage("Spoke: I've moved forward.")).toEqual({
      kind: "spoke",
      text: "I've moved forward.",
    });
  });

  it("classifies warnings and strips the prefix", () => {
    expect(
      classifyAgentMessage("Warning: TTS timeout while speaking: hi"),
    ).toEqual({ kind: "warning", text: "TTS timeout while speaking: hi" });
  });

  it("classifies completion status", () => {
    expect(
      classifyAgentMessage("'Hello' command executed successfully.")!.kind,
    ).toBe("status");
    expect(classifyAgentMessage("Navigation goal reached")!.kind).toBe("status");
  });

  it("strips wrapping quotes around the whole string", () => {
    expect(classifyAgentMessage("'Hello there'")!.text).toBe("Hello there");
  });

  it("treats other text as a message", () => {
    expect(classifyAgentMessage("Hello! I am Daneel.")).toEqual({
      kind: "message",
      text: "Hello! I am Daneel.",
    });
  });

  it("extracts text from a JSON frame", () => {
    expect(classifyAgentMessage('{"text":"hi there"}')).toEqual({
      kind: "message",
      text: "hi there",
    });
  });
});
