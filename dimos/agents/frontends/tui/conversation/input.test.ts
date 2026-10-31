import assert from "node:assert/strict";
import { ChatInput, PromptEditor } from "./input.ts";
import { ProcessTerminal, TuiMainScreen } from "@earendil-works/pi-tui";
import { initTheme } from "@earendil-works/pi-coding-agent";
import { test } from "node:test";

test("Pi editor preserves multiline submissions and never puts credentials into chat history", () => {
  initTheme("dark", false);
  const editor = new PromptEditor(new TuiMainScreen(new ProcessTerminal()), ["help"], process.cwd());
  let submitted = "";
  editor.onSubmit = (value) => {
    submitted = value;
  };
  editor.handleInput("\x1b[200~line one\nline two\x1b[201~");
  editor.handleInput("\r");
  assert.equal(submitted, "line one\nline two");
  for (const key of ["\x1b[13;2u", "\x1b[27;2;13~", "\x1b[13;2~", "\x1b\r", "\n"]) {
    editor.setValue("first");
    submitted = "";
    editor.handleInput(key);
    editor.handleInput("second");
    assert.equal(submitted, "", "modified Enter must not submit");
    assert.equal(editor.getValue(), "first\nsecond");
    editor.handleInput("\r");
    assert.equal(submitted, "first\nsecond", "plain Enter submits both lines");
  }
  editor.setValue("");
  editor.secret = true;
  editor.setValue("");
  editor.handleInput("private-fixture");
  assert(!editor.render(50).join("").includes("private-fixture"));
  editor.secret = false;
  editor.setValue("");
  for (const key of ["\x1f", "\x19", "\x1b[A"]) editor.handleInput(key);
  assert(!editor.getValue().includes("private-fixture"));
});

test("credentials cannot return through input undo after auth", () => {
  const input = new ChatInput();
  input.secret = true;
  input.handleInput("fixture-secret");
  assert(!input.render(80).join("").includes("fixture-secret"));
  input.secret = false;
  input.handleInput("\x1f");
  input.handleInput("\x19");
  assert.equal(input.getValue(), "");
});
