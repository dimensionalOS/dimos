import assert from "node:assert/strict";
import { ChatInput, PromptEditor } from "#dimos/agents/frontends/tui/conversation/input.ts";
import { ProcessTerminal, TuiMainScreen } from "@earendil-works/pi-tui";
import { initTheme } from "@earendil-works/pi-coding-agent";
import { test } from "node:test";
import { mkdir, mkdtemp, rm, writeFile } from "node:fs/promises";
import { homedir, tmpdir } from "node:os";
import { join, relative } from "node:path";
import { setTimeout as delay } from "node:timers/promises";
import type { StreamRef } from "#dimos/agents/shared/protocol/streams.ts";
import { StreamRefSchema } from "#dimos/agents/shared/protocol/streams.ts";
import { Value } from "typebox/value";

function stream(channel: string, source: unknown = null): StreamRef {
  return Value.Parse(StreamRefSchema, { channel, encoding: "image.jpeg.v1", source });
}

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

test("@ offers files before live streams and keeps the catalog callback across workspace changes", async (t) => {
  const root = await mkdtemp(join(tmpdir(), "dimos-stream-completion-"));
  t.after(() => rm(root, { recursive: true, force: true }));
  await writeFile(join(root, "camera.txt"), "fixture");
  await mkdir(join(root, "other"));
  await writeFile(join(root, "other", "other.txt"), "fixture");
  let streams = [
    stream("camera", { module: "PeopleFaceMasks", name: "people_faces_overlay", direction: "out", type: "Image" }),
  ];
  initTheme("dark", false);
  const editor = new PromptEditor(new TuiMainScreen(new ProcessTerminal()), ["help"], root, () => streams);
  const open = async (): Promise<void> => {
    editor.handleInput("\t");
    await delay(0);
  };
  const complete = async (text: string): Promise<void> => {
    editor.setValue(text);
    await open();
    if (editor.isShowingAutocomplete()) editor.handleInput("\t");
    await delay(0);
  };

  editor.setValue("inspect @");
  await open();
  assert(editor.isShowingAutocomplete());
  const picker = editor.render(80).join("\n");
  assert(picker.includes("camera.txt"));
  assert(picker.includes("PeopleFaceMasks.people_faces_overlay  Image"));
  assert(!picker.includes("image.jpeg.v1"));
  editor.handleInput("\x1b");
  editor.setValue("inspect @cam");
  await open();
  editor.handleInput("\t");
  assert.equal(editor.getValue(), "inspect camera.txt ");

  editor.setValue("inspect @cam");
  await open();
  editor.handleInput("\x1b[B");
  editor.handleInput("\t");
  assert.equal(editor.getValue(), "inspect stream://camera ");
  await complete("@PeopleFaceMasks.people");
  assert.equal(editor.getValue(), "stream://camera ");
  await complete("@stream://cam");
  assert.equal(editor.getValue(), "stream://camera ");
  await complete("@stream://");
  assert.equal(editor.getValue(), "stream://camera ");

  streams = [stream("new_channel", { module: "vision", name: "color", direction: "out", type: "Image" })];
  for (const query of ["@new", "@vision", "@color", "@Image", "@jpeg"]) {
    await complete(query);
    assert.equal(editor.getValue(), "stream://new_channel ");
  }
  editor.setWorkspace(join(root, "other"));
  await complete("@new");
  assert.equal(editor.getValue(), "stream://new_channel ");
  await complete("@other");
  assert.equal(editor.getValue(), "other.txt ");
  await complete("/he");
  assert.equal(editor.getValue(), "/help ");

  streams = [];
  await complete("@new");
  assert.equal(editor.getValue(), "@new");
  await complete("@other");
  assert.equal(editor.getValue(), "other.txt ");
});

test("stream selections preserve exact channel IDs as passive ordinary text", async (t) => {
  const root = await mkdtemp(join(tmpdir(), "dimos-stream-text-"));
  t.after(() => rm(root, { recursive: true, force: true }));
  let streams: StreamRef[] = [];
  initTheme("dark", false);
  const editor = new PromptEditor(new TuiMainScreen(new ProcessTerminal()), [], root, () => streams);
  let submitted = "";
  editor.onSubmit = (value) => {
    submitted = value;
  };
  for (const [channel, reference] of [
    ["Color_image/v1", "stream://Color_image/v1"],
    ["camera%2F%25front", "stream://camera%2F%25front"],
    ["stream://camera", "stream://stream://camera"],
    ["camera/", "stream://camera/"],
    ["camera front", '"stream://camera front"'],
    ['camera"front', '"stream://camera\\"front"'],
    ["camera\\front", '"stream://camera\\\\front"'],
    ["camera=front", '"stream://camera=front"'],
    ["camera(front)", '"stream://camera(front)"'],
    ["camera'front", '"stream://camera\'front"'],
    ["camera&front", '"stream://camera&front"'],
    ["camera\tfront", '"stream://camera\\tfront"'],
    ["camera\nfront", '"stream://camera\\nfront"'],
    ["camera\x1b[31mfront", '"stream://camera\\u001b[31mfront"'],
    ["camera\ud800front", '"stream://camera\\ud800front"'],
  ]) {
    streams = [stream(channel)];
    editor.setValue("inspect @stream://");
    editor.handleInput("\t");
    await delay(0);
    if (editor.isShowingAutocomplete()) editor.handleInput("\t");
    assert.equal(editor.getValue(), `inspect ${reference} `);
    const recovered: unknown = reference.startsWith('"') ? JSON.parse(reference) : reference;
    assert(typeof recovered === "string");
    assert.equal(recovered.slice("stream://".length), channel);
    editor.handleInput("\r");
    assert.equal(submitted, `inspect ${reference}`);
  }

  streams = [stream("camera/"), stream("camera\x1b[31m\nfront")];
  editor.setValue("@cam");
  editor.handleInput("\t");
  await delay(0);
  const picker = editor.render(120).join("\n");
  assert(picker.includes("camera.  [stream]"));
  assert(picker.includes("camera [31m front  [stream]"));
  assert(!picker.includes("\x1b[31m"));
  editor.handleInput("\t");
  assert.equal(editor.getValue(), "stream://camera/ ");

  for (const [channel, typed, expected] of [
    ["camera front", 'before\ninspect @"camera front" after', 'before\ninspect "stream://camera front"  after'],
    ["camera", 'before\ninspect @"cam" after', 'before\ninspect "stream://camera"  after'],
  ]) {
    streams = [stream(channel)];
    editor.setValue(typed);
    for (let i = 0; i < '" after'.length; i++) editor.handleInput("\x1b[D");
    editor.handleInput("\t");
    await delay(0);
    if (editor.isShowingAutocomplete()) editor.handleInput("\t");
    assert.equal(editor.getValue(), expected);
  }
});
