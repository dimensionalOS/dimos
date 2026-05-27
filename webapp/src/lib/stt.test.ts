import { describe, it, expect } from "vitest";
import { wrapUserSpeech } from "./stt";

describe("wrapUserSpeech", () => {
  it("wraps the transcript in <user_speech> tags", () => {
    expect(wrapUserSpeech("stand up")).toBe(
      "<user_speech>stand up</user_speech>",
    );
  });

  it("trims surrounding whitespace before wrapping", () => {
    expect(wrapUserSpeech("  go to the kitchen \n")).toBe(
      "<user_speech>go to the kitchen</user_speech>",
    );
  });
});
