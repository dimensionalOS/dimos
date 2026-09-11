import { describe, it, expect } from "vitest";
import { displayText, inlineTokens } from "./terminalText.ts";
describe("humancli presentation", () => {
  it("omits transport markers while retaining an actual reply", () => {
    expect(displayText("[reasoning]\n[function_call]\nI am here")).toBe("I am here");
    expect(displayText("The literal [reasoning] appears in this sentence.")).toContain("[reasoning]");
  });
  it("formats the reported coordinate message and preserves unsafe HTML as text", () => {
    expect(inlineTokens("At **(-1.15, 1.38)**, run `where_am_i()`" )).toEqual([
      {kind:"text",text:"At "},{kind:"strong",text:"(-1.15, 1.38)"},{kind:"text",text:", run "},{kind:"code",text:"where_am_i()"}
    ]);
    expect(inlineTokens('<img src=x onerror="alert(1)">')).toEqual([{kind:"text",text:'<img src=x onerror="alert(1)">'}]);
  });
});
