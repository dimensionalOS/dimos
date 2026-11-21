import assert from "node:assert/strict";
import { test } from "node:test";
import { stripVTControlCharacters } from "node:util";
import { CURSOR_MARKER, ProcessTerminal, TuiMainScreen, visibleWidth } from "@earendil-works/pi-tui";
import { initTheme } from "@earendil-works/pi-coding-agent";
import {
  GENERAL_PROFILE,
  type Profile,
  type ProfileDraft,
  type ProfileOptions,
} from "#dimos/agents/shared/protocol/profiles.ts";
import { ProfileEditor } from "#dimos/agents/frontends/tui/menu/lib/profile-editor.ts";

initTheme("dark", false);
const tui = new TuiMainScreen(new ProcessTerminal());
const choices: ProfileOptions = {
  blueprints: ["unitree-go2-agentic", "xarm-grasp-agent"],
  skills: [
    { name: "snacks", description: "Sort mixed snacks" },
    { name: "safety", description: "Safe handling" },
  ],
  blueprintsError: null,
};

function fixture(profile: Profile | null = null) {
  const saved: ProfileDraft[] = [];
  let cancelled = false;
  const editor = new ProfileEditor({
    tui,
    profile,
    save: (value) => saved.push(value),
    cancel: () => {
      cancelled = true;
    },
  });
  editor.setOptions(choices);
  return { editor, saved, cancelled: () => cancelled };
}

function text(editor: ProfileEditor, width = 60, height = 24): string {
  const lines = editor.render(width, height);
  assert(lines.length <= height);
  assert(lines.every((line) => visibleWidth(line) <= width));
  return stripVTControlCharacters(lines.join("\n"));
}

function tabs(editor: ProfileEditor, count: number): void {
  for (let i = 0; i < count; i++) editor.handleInput("\t");
}

function type(editor: ProfileEditor, value: string): void {
  for (const character of value) editor.handleInput(character);
}

function done(editor: ProfileEditor): void {
  editor.handleInput("\x1b");
}

test("one form shows exactly the six fields and buttons at wide and narrow sizes", () => {
  const { editor } = fixture();
  for (const [width, height] of [
    [100, 32],
    [60, 24],
  ]) {
    const rendered = text(editor, width, height);
    for (const label of [
      "Name *",
      "Description",
      "Role",
      "Blueprints",
      "Available skills",
      "Always loaded skills",
      "[ Cancel ]",
      "[ Create profile ]",
    ])
      assert(rendered.includes(label), `${label} at ${width}x${height}`);
    for (const removed of ["Robot", "Workspace", "Configuration", "Startup", "Modules", "JSON", "ID", "1/12"])
      assert(!rendered.includes(removed));
  }
});

test("new profiles contain six fields without an id and role supports multiline editing", () => {
  const { editor, saved } = fixture();
  type(editor, "Piper");
  tabs(editor, 1);
  type(editor, "Snack sorting");
  tabs(editor, 1);
  type(editor, "Sort");
  editor.handleInput("\r");
  type(editor, "snacks");
  editor.handleInput("\x13");
  assert.deepEqual(saved, [
    {
      name: "Piper",
      description: "Snack sorting",
      role: "Sort\nsnacks",
      blueprints: [],
      availableSkills: [],
      alwaysLoadedSkills: [],
    },
  ]);
});

test("editing retains the hidden id and ordinary text cursor editing", () => {
  const profile = { ...GENERAL_PROFILE, id: "hidden-id", name: "Piper", role: "Sort\nsnacks" };
  const { editor, saved } = fixture(profile);
  assert(!text(editor).includes(profile.id));
  editor.handleInput("\x1b[D");
  type(editor, "!");
  editor.handleInput("\x13");
  assert.deepEqual(saved, [{ ...profile, name: "Pipe!r" }]);
});

test("validation stays next to the name and focuses it on a short screen", () => {
  const { editor, saved } = fixture();
  tabs(editor, 7);
  editor.handleInput("\r");
  const rendered = text(editor, 60, 16);
  assert.match(rendered, /Name \*[\s\S]*Enter a name\./);
  assert(rendered.includes("[ Create profile ]"));
  assert.equal(saved.length, 0);
  type(editor, "Piper");
  assert(!text(editor).includes("Enter a name."));
  editor.handleInput("\x13");
  assert.equal(saved[0]?.name, "Piper");
});

test("Enter toggles without leaving the picker and Escape keeps selections in the form with exclusive skill modes", () => {
  const { editor, saved } = fixture({ ...GENERAL_PROFILE, id: "piper", alwaysLoadedSkills: ["snacks"] });
  tabs(editor, 4);
  editor.handleInput("\r");
  type(editor, "mixed");
  assert(text(editor).includes("snacks"));
  assert(!text(editor).includes("Safe handling"));
  editor.handleInput("\r");
  assert(text(editor).includes("[x] snacks"));
  assert(!text(editor).includes("Edit profile"));
  editor.handleInput("\r");
  assert(text(editor).includes("[ ] snacks"));
  editor.handleInput(" ");
  editor.handleInput("\x1b");
  assert(text(editor).includes("1 selected · snacks"));
  editor.handleInput("\x13");
  assert.deepEqual(saved[0]?.availableSkills, ["snacks"]);
  assert.deepEqual(saved[0]?.alwaysLoadedSkills, []);
});

test("saved choices remain editable when catalogs fail and escape returns one level", () => {
  const f = fixture({ ...GENERAL_PROFILE, id: "piper", blueprints: ["offline-blueprint"] });
  f.editor.setOptionsError("Connection lost");
  tabs(f.editor, 3);
  f.editor.handleInput("\r");
  assert(text(f.editor).includes("Connection lost"));
  assert(text(f.editor).includes("[x] offline-blueprint"));
  f.editor.handleInput("\x1b");
  assert(!f.cancelled());
  assert(text(f.editor).includes("Edit profile"));
  f.editor.handleInput("\x1b");
  assert(f.cancelled());
});

test("picker accepts catalogs arriving after opening and navigates beyond nine choices", () => {
  const { editor, saved } = fixture({ ...GENERAL_PROFILE, id: "piper" });
  tabs(editor, 3);
  editor.handleInput("\r");
  editor.setOptions({ ...choices, blueprints: Array.from({ length: 20 }, (_, i) => `blueprint-${i}`) });
  for (let i = 0; i < 12; i++) editor.handleInput("\x1b[B");
  editor.handleInput(" ");
  assert(text(editor).includes("[x] blueprint-12"));
  done(editor);
  editor.handleInput("\x13");
  assert.deepEqual(saved[0]?.blueprints, ["blueprint-12"]);
});

test("long role keeps its cursor and form actions visible on a short screen", () => {
  const { editor } = fixture({
    ...GENERAL_PROFILE,
    role: Array.from({ length: 30 }, (_, i) => `Instruction ${i}`).join("\n"),
  });
  tabs(editor, 2);
  const lines = editor.render(60, 16);
  assert(lines.some((line) => line.includes(CURSOR_MARKER)));
  assert(text(editor, 60, 16).includes("[ Save changes ]"));
  for (let i = 0; i < 20; i++) editor.handleInput("\x1b[A");
  assert(editor.render(60, 16).some((line) => line.includes(CURSOR_MARKER)));
});

test("failed save retains values and allows a retry while duplicate submissions are blocked", () => {
  const { editor, saved } = fixture();
  type(editor, "Piper");
  editor.handleInput("\x13");
  editor.handleInput("\x13");
  assert.equal(saved.length, 1);
  editor.setError("A profile with this name already exists.");
  assert(text(editor).includes("already exists"));
  type(editor, " 2");
  editor.handleInput("\x13");
  assert.equal(saved[1]?.name, "Piper 2");
});

test("Up and Down move through role lines and cross fields only at the text boundaries", () => {
  const { editor } = fixture({ ...GENERAL_PROFILE, role: "First\nSecond\nThird" });
  editor.render(60, 24);
  editor.handleInput("\x1b[B");
  assert.match(text(editor), /› Description/);
  editor.handleInput("\x1b[B");
  assert.match(text(editor), /› Role/);
  for (let i = 0; i < 2; i++) {
    editor.handleInput("\x1b[A");
    assert.match(text(editor), /› Role/);
  }
  editor.handleInput("\x1b[A");
  assert.match(text(editor), /› Description/);
  editor.handleInput("\x1b[B");
  assert.match(text(editor), /› Role/);
  for (let i = 0; i < 2; i++) {
    editor.handleInput("\x1b[B");
    assert.match(text(editor), /› Role/);
  }
  editor.handleInput("\x1b[B");
  assert.match(text(editor), /› Blueprints/);
});

test("wrapped role lines keep arrow focus until the first or last visual line", () => {
  const { editor } = fixture({
    ...GENERAL_PROFILE,
    role: "A long role instruction that wraps onto several visual lines in a narrow terminal.",
  });
  tabs(editor, 2);
  text(editor, 30, 24);
  editor.handleInput("\x1b[A");
  assert.match(text(editor, 30, 24), /› Role/);
  editor.handleInput("\x1b[B");
  assert.match(text(editor, 30, 24), /› Role/);
  editor.handleInput("\x1b[B");
  assert.match(text(editor, 30, 24), /› Blueprints/);
});
