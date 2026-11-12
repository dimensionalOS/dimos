import assert from "node:assert/strict";
import { ChatInput, PromptEditor } from "#dimos/agents/frontends/tui/conversation/input.ts";
import { ProcessTerminal, TuiMainScreen } from "@earendil-works/pi-tui";
import { initTheme } from "@earendil-works/pi-coding-agent";
import { test } from "node:test";
import { mkdir, mkdtemp, rm, writeFile } from "node:fs/promises";
import { homedir, tmpdir } from "node:os";
import { join, relative } from "node:path";
import { setTimeout as delay } from "node:timers/promises";

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

test("@ selection inserts ordinary paths and follows the conversation workspace", async (t) => {
  const root = await mkdtemp(join(tmpdir(), "dimos-path-completion-"));
  t.after(() => rm(root, { recursive: true, force: true }));
  for (const directory of ["nested", "quoted dir", "equal=dir", "other"]) await mkdir(join(root, directory));
  for (const file of [
    "alpha.txt",
    "with spaces.txt",
    "nested/child.txt",
    "quoted dir/child.txt",
    "equal=dir/child.txt",
    "other/another.txt",
  ])
    await writeFile(join(root, file), "fixture");
  initTheme("dark", false);
  const editor = new PromptEditor(new TuiMainScreen(new ProcessTerminal()), ["help"], root);
  const complete = async (): Promise<void> => {
    editor.handleInput("\t");
    await delay(0);
    if (editor.isShowingAutocomplete()) editor.handleInput("\t");
    await delay(0);
  };
  for (const [typed, expected] of [
    ["inspect @al", "inspect alpha.txt "],
    ["@./al", "./alpha.txt "],
    [`@${root}/al`, `${root}/alpha.txt `],
    [`@~/${relative(homedir(), root)}/al`, `~/${relative(homedir(), root)}/alpha.txt `],
    ["@with", '"with spaces.txt" '],
    ['@"al', '"alpha.txt" '],
  ]) {
    editor.setValue(typed);
    await complete();
    assert.equal(editor.getValue(), expected);
  }
  for (const [typed, directory, expected] of [
    ["@nes", "nested/", "nested/child.txt"],
    ["@quo", '"quoted dir/"', '"quoted dir/child.txt"'],
    ["@equ", '"equal=dir/"', '"equal=dir/child.txt"'],
  ]) {
    editor.setValue(typed);
    await complete();
    assert.equal(editor.getValue(), directory);
    editor.handleInput("ch");
    await complete();
    assert.equal(editor.getValue(), expected);
  }
  editor.setValue('before\ninspect @"al" after');
  for (let i = 0; i < '" after'.length; i++) editor.handleInput("\x1b[D");
  await complete();
  assert.equal(editor.getValue(), 'before\ninspect "alpha.txt"  after');

  for (const literal of ["@missing/ref", "person@al", '@"alpha.txt"']) {
    editor.setValue(literal);
    await complete();
    assert.equal(editor.getValue(), literal);
  }
  editor.setWorkspace(join(root, "other"));
  editor.setValue("@an");
  await complete();
  assert.equal(editor.getValue(), "another.txt ");
  editor.setValue("/he");
  await complete();
  assert.equal(editor.getValue(), "/help ");

  let submitted = "";
  editor.onSubmit = (value) => {
    submitted = value;
  };
  editor.setValue("inspect @an");
  await complete();
  editor.handleInput("@missing/ref");
  editor.handleInput("\r");
  assert.equal(submitted, "inspect another.txt @missing/ref");
});
