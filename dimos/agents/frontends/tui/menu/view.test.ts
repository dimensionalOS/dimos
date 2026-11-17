import assert from "node:assert/strict";
import { test } from "node:test";
import { ProcessTerminal, TuiMainScreen, visibleWidth } from "@earendil-works/pi-tui";
import { initTheme } from "@earendil-works/pi-coding-agent";
import { Status } from "#dimos/agents/shared/protocol/status.ts";
import {
  GENERAL_PROFILE,
  type Profile,
  type ProfileCatalog,
  type ProfileDraft,
} from "#dimos/agents/shared/protocol/profiles.ts";
import { MenuView, newMenuMemory } from "#dimos/agents/frontends/tui/menu/view.ts";
import { Feedback } from "#dimos/agents/frontends/tui/feedback.ts";
import type { Ui } from "#dimos/agents/frontends/tui/session.ts";
import type { JobRow } from "#dimos/agents/shared/protocol/jobs/state.ts";
import type { ModuleRow } from "#dimos/agents/shared/protocol/runtime/module.ts";
import type { ConversationRow } from "#dimos/agents/shared/protocol/conversation/summary.ts";
import { parseJobName } from "#dimos/agents/harness/jobs/registry.ts";
import { parseTimeZone } from "#dimos/agents/harness/jobs/cron.ts";
import { AgentModuleNameSchema, ConversationIdSchema } from "#dimos/agents/shared/protocol/args.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { Value } from "typebox/value";
import { stripVTControlCharacters } from "node:util";

const state = <T>(value: T) =>
  new Status(value, (error) => {
    throw error;
  });
const tui = new TuiMainScreen(new ProcessTerminal());
const keys = { down: "\x1b[B", up: "\x1b[A", right: "\x1b[C", left: "\x1b[D", enter: "\r", escape: "\x1b" };
const tick = () => new Promise((resolve) => setImmediate(resolve));
initTheme("dark", false);

function fixture(
  profiles: Profile[],
  options: {
    jobs?: readonly JobRow[];
    modules?: readonly ModuleRow[];
    conversations?: readonly ConversationRow[];
    refuseDefault?: boolean;
  } = {},
) {
  const catalog = state<ProfileCatalog>({ profiles, defaultProfile: null });
  const calls: string[] = [];
  const ended = new Promise<never>(() => {});
  const actions = new Feedback(ended);
  const gateway = {
    variant: "owner",
    link: state({ state: "up" }),
    load: state({ state: "idle" }),
    runs: state([]),
    conversations: state(options.conversations ?? []),
    profiles: catalog,
    modules: state(options.modules ?? []),
    jobs: state(options.jobs ?? []),
    profileOptions: async () => ({ ok: true, value: { blueprints: [], skills: [], blueprintsError: null } }),
    saveProfile: async ({ profile }: { profile: ProfileDraft }) => {
      const saved = { ...profile, id: profile.id ?? "new-profile" };
      catalog.set({
        ...catalog.value,
        profiles: [...catalog.value.profiles.filter((row) => row.id !== saved.id), saved].sort((a, b) =>
          a.name.localeCompare(b.name),
        ),
      });
      return { ok: true, value: saved };
    },
    deleteProfile: async ({ profile }: { profile: string }) => {
      calls.push(`delete ${profile}`);
      catalog.set({ ...catalog.value, profiles: catalog.value.profiles.filter((entry) => entry.id !== profile) });
      return { ok: true, value: null };
    },
    setDefaultProfile: async ({ profile }: { profile: string }) => {
      calls.push(`default ${profile}`);
      return options.refuseDefault
        ? { ok: false, refusal: { code: "invalid", field: "profile", problem: "Cannot set this default." } }
        : { ok: true, value: null };
    },
    restartModule: async ({ name }: { name: string }) => {
      calls.push(`restart ${name}`);
      return { ok: true, value: null };
    },
    unloadModule: async ({ name }: { name: string }) => {
      calls.push(`unload ${name}`);
      return { ok: true, value: null };
    },
    deleteModule: async ({ name }: { name: string }) => {
      calls.push(`delete ${name}`);
      return { ok: true, value: null };
    },
    runJob: async ({ name }: { name: string }) => {
      calls.push(`run ${name}`);
      return { ok: true, value: null };
    },
    deleteJob: async ({ name }: { name: string }) => {
      calls.push(`delete ${name}`);
      return { ok: true, value: null };
    },
  };
  const ui = { gateway, actions, ended, connection: state({ state: "live", since: epochMs(0) }) } as unknown as Ui;
  let created = "";
  let attached = 0;
  let closed = false;
  const memory = newMenuMemory();
  const open = () =>
    new MenuView({
      ui,
      tui,
      memory,
      current: null,
      attach: (id) => {
        attached = id;
      },
      create: (id) => {
        created = id;
      },
      close: () => {
        closed = true;
      },
      requestRender: () => {},
      height: () => 25,
    });
  return {
    open,
    memory,
    catalog,
    gateway,
    calls,
    actions,
    created: () => created,
    attached: () => attached,
    closed: () => closed,
  };
}

function text(menu: MenuView, width = 120): string {
  const lines = menu.render(width);
  assert.equal(lines.length, 25);
  assert(lines.every((line) => visibleWidth(line) <= width));
  assert.match(stripVTControlCharacters(lines.at(-1) ?? ""), /gateway.*DimOS/);
  return stripVTControlCharacters(lines.join("\n"));
}
function type(menu: MenuView, value: string): void {
  for (const character of value) menu.handleInput(character);
}
function presses(menu: MenuView, key: string, count = 1): void {
  for (let i = 0; i < count; i++) menu.handleInput(key);
}
function conversation(id: number, profile = GENERAL_PROFILE): ConversationRow {
  return {
    id: Value.Parse(ConversationIdSchema, id),
    profile,
    parent: null,
    title: `Conversation ${id}`,
    cwd: null,
    running: null,
    queued: 0,
    unread: 0,
    costUsd: 0,
    updatedAt: epochMs(id),
  };
}
const piper = { ...GENERAL_PROFILE, id: "piper", name: "Piper", role: "Sort snacks" };

for (const width of [60, 120]) {
  test(`profile opens definition and vertical actions, using arrows without Tab at width ${width}`, async (t) => {
    const f = fixture([GENERAL_PROFILE, piper], { conversations: [conversation(2, piper), conversation(3, piper)] });
    const menu = f.open();
    t.after(() => menu.dispose());
    presses(menu, keys.down, 2);
    menu.handleInput(keys.enter);
    const rendered = text(menu, width);
    assert.match(rendered, /Profiles › Piper/);
    assert.match(rendered, /Sort snacks/);
    assert.match(
      rendered,
      /› Start new conversation\n\s+Resume latest conversation #3\n\s+Edit profile\n\s+Set as default/,
    );
    assert(!rendered.includes("Setup"));
    menu.handleInput(keys.right);
    assert.match(text(menu, width), /› Start new conversation/);
    menu.handleInput(keys.down);
    menu.handleInput(keys.enter);
    await tick();
    assert.equal(f.attached(), 3);
    presses(menu, keys.down, 2);
    menu.handleInput(keys.enter);
    await tick();
    assert.deepEqual(f.calls, ["default piper"]);
    presses(menu, keys.up, 3);
    menu.handleInput(keys.enter);
    await tick();
    assert.equal(f.created(), "piper");
  });
}

test("menu filters beyond nine profiles and restores the selected list and search on Escape", async (t) => {
  const profiles = [
    GENERAL_PROFILE,
    ...Array.from({ length: 32 }, (_, i) => ({ ...piper, id: `robot-${i}`, name: `Robot ${i}` })),
  ];
  const f = fixture(profiles);
  const menu = f.open();
  t.after(() => menu.dispose());
  type(menu, "Robot ");
  presses(menu, keys.down, 26);
  const before = text(menu, 60);
  assert.match(before, /› Robot 25/);
  menu.handleInput(keys.enter);
  menu.handleInput(keys.escape);
  assert.equal(text(menu, 60), before);
  menu.dispose();
  const reopened = f.open();
  t.after(() => reopened.dispose());
  assert.equal(text(reopened, 60), before);
  reopened.handleInput(keys.enter);
  reopened.handleInput(keys.enter);
  await tick();
  assert.equal(f.created(), "robot-25");
  reopened.handleInput(keys.escape);
  assert(!f.closed());
  reopened.handleInput(keys.escape);
  assert(f.closed());
});

test("terminal-encoded printable keys filter each section and preserve its query", (t) => {
  const f = fixture([GENERAL_PROFILE, { ...piper, name: "Bench_A" }]);
  const menu = f.open();
  t.after(() => menu.dispose());
  for (let section = 0; section < 4; section++) {
    menu.handleInput("\x1b[45:95;2u");
    menu.handleInput("\x1b[97u");
    assert.match(text(menu), /Search: _a/);
    if (section === 0) assert.match(text(menu), /Bench_A/);
    menu.handleInput(keys.right);
  }
  assert.match(text(menu), /Search: _a/);
  menu.handleInput("\x1b[127u");
  assert.match(text(menu), /Search: _\n/);
  menu.handleInput("\x1b[128295u");
  assert.match(text(menu), /Search: _🔧/);
  menu.handleInput("\x1b[127u");
  assert.match(text(menu), /Search: _\n/);
});

test("Create profile is selectable with empty catalogs or a search with no matches; cancel restores the list", async (t) => {
  for (const profiles of [[], [GENERAL_PROFILE]]) {
    const f = fixture(profiles);
    const menu = f.open();
    t.after(() => menu.dispose());
    type(menu, "no such profile");
    const before = text(menu, 60);
    assert.match(before, /› \+ Create profile/);
    menu.handleInput(keys.enter);
    await tick();
    assert.match(stripVTControlCharacters(menu.render(60).join("\n")), /Name \*/);
    menu.handleInput(keys.escape);
    assert.equal(text(menu, 60), before);
    assert(!f.closed());
  }
});

test("editing returns to the same profile with updated values; create selects the saved profile", async (t) => {
  const f = fixture([GENERAL_PROFILE, piper]);
  const menu = f.open();
  t.after(() => menu.dispose());
  presses(menu, keys.down, 2);
  menu.handleInput(keys.enter);
  menu.handleInput(keys.down);
  menu.handleInput(keys.enter);
  await tick();
  menu.handleInput("\x01");
  type(menu, "A ");
  menu.handleInput("\x13");
  await tick();
  assert.match(text(menu), /A Piper/);
  assert.match(text(menu), /› Edit profile/);
  menu.handleInput(keys.escape);
  assert.match(text(menu), /› A Piper/);
  presses(menu, keys.up, 2);
  menu.handleInput(keys.enter);
  await tick();
  type(menu, "New robot");
  menu.handleInput("\x13");
  await tick();
  assert.match(text(menu), /› New robot/);
  menu.handleInput(keys.enter);
  menu.handleInput(keys.enter);
  await tick();
  assert.equal(f.created(), "new-profile");
});

test("long roles keep blueprints, both skill modes, and actions visible", (t) => {
  const f = fixture([
    GENERAL_PROFILE,
    {
      ...piper,
      role: "Group snacks by type and arrange them neatly. ".repeat(18),
      blueprints: ["piper-teleop"],
      alwaysLoadedSkills: ["sorting"],
      availableSkills: ["presentation"],
    },
  ]);
  const menu = f.open();
  t.after(() => menu.dispose());
  presses(menu, keys.down, 2);
  menu.handleInput(keys.enter);
  for (const width of [60, 120]) {
    const rendered = text(menu, width);
    assert.match(rendered, /Expected blueprints: piper-teleop/);
    assert.match(rendered, /Always-loaded skills: sorting/);
    assert.match(rendered, /Available skills: presentation/);
    assert.match(rendered, /Edit profile/);
    assert.match(rendered, /Set as default/);
  }
});

test("a created profile is selected when its catalog publication follows the save response", async (t) => {
  const f = fixture([GENERAL_PROFILE]);
  const menu = f.open();
  t.after(() => menu.dispose());
  const saved = { ...piper, id: "new-profile", name: "New robot" };
  f.gateway.saveProfile = async () => ({ ok: true, value: saved });
  menu.handleInput(keys.enter);
  await tick();
  type(menu, "New robot");
  menu.handleInput("\x13");
  await tick();
  text(menu);
  f.catalog.set({ profiles: [GENERAL_PROFILE, saved], defaultProfile: null });
  assert.match(text(menu), /› New robot/);
});

test("profile identity survives catalog reorder and removal without acting on a different profile", async (t) => {
  const f = fixture([GENERAL_PROFILE, piper]);
  const menu = f.open();
  t.after(() => menu.dispose());
  presses(menu, keys.down, 2);
  menu.handleInput(keys.enter);
  f.catalog.set({ profiles: [piper, GENERAL_PROFILE], defaultProfile: null });
  menu.handleInput(keys.enter);
  await tick();
  assert.equal(f.created(), "piper");
  f.catalog.set({ profiles: [GENERAL_PROFILE], defaultProfile: null });
  assert.match(text(menu), /This item is gone/);
  menu.handleInput(keys.enter);
  await tick();
  assert.equal(f.created(), "piper");
});

test("deleting a custom profile requires confirming the selected profile", async (t) => {
  const f = fixture([GENERAL_PROFILE, piper]);
  const menu = f.open();
  t.after(() => menu.dispose());
  presses(menu, keys.down, 2);
  menu.handleInput(keys.enter);
  presses(menu, keys.down, 3);
  menu.handleInput(keys.enter);
  await tick();
  assert.match(text(menu), /› Cancel/);
  assert.deepEqual(f.calls, []);
  menu.handleInput(keys.down);
  menu.handleInput(keys.enter);
  await tick();
  assert.deepEqual(f.calls, ["delete piper"]);
  assert.match(text(menu), /Search:/);
});

test("default action refusal is shown beside the vertical actions", async (t) => {
  const f = fixture([GENERAL_PROFILE], { refuseDefault: true });
  const menu = f.open();
  t.after(() => menu.dispose());
  menu.handleInput(keys.down);
  menu.handleInput(keys.enter);
  assert(!text(menu).includes("Edit profile"));
  assert(!text(menu).includes("Delete profile"));
  menu.handleInput(keys.down);
  menu.handleInput(keys.enter);
  await tick();
  assert.match(text(menu), /Cannot set this default/);
});

test("Conversations Enter resumes immediately", async (t) => {
  const f = fixture([GENERAL_PROFILE], { conversations: [conversation(8)] });
  const menu = f.open();
  t.after(() => menu.dispose());
  menu.handleInput(keys.right);
  menu.handleInput(keys.enter);
  await tick();
  assert.equal(f.attached(), 8);
});

test("module actions open vertically and execute the selected action with live details", async (t) => {
  const module: ModuleRow = {
    name: Value.Parse(AgentModuleNameSchema, "snack_sorter"),
    class: "SnackSorter",
    owner: null,
    path: "/tmp/snack_sorter.py",
    lastError: null,
    state: "running",
    revision: 1,
    instance: null,
  };
  const f = fixture([GENERAL_PROFILE], { modules: [module] });
  const menu = f.open();
  t.after(() => menu.dispose());
  presses(menu, keys.right, 2);
  menu.handleInput(keys.enter);
  assert.match(text(menu), /Agent modules › snack_sorter/);
  assert.match(text(menu), /snack_sorter.*r1 running/);
  assert.match(text(menu), /› Restart\n\s+Unload\n\s+Delete/);
  for (const [index, expected] of ["restart", "unload", "delete"].entries()) {
    if (index > 0) menu.handleInput(keys.down);
    menu.handleInput(keys.enter);
    await tick();
    assert.equal(f.calls.at(-1), `${expected} snack_sorter`);
  }
  f.gateway.modules.set([{ ...module, state: "stopped", revision: 2 }]);
  assert.match(text(menu), /r2 stopped/);
  menu.handleInput(keys.escape);
  assert.match(text(menu), /Search:/);
});

test("Jobs filter by full names, open actions before history, and return one level at a time", async (t) => {
  const job: JobRow = {
    name: parseJobName("daily-camera-calibration"),
    cron: "0 0 * * *",
    timeZone: parseTimeZone("UTC"),
    action: { kind: "notify", text: "Calibrate." },
    conversation: Value.Parse(ConversationIdSchema, 1),
    createdAt: epochMs(0),
    runs: Array.from({ length: 24 }, (_, i) => ({
      run: { kind: "scheduled", due: epochMs(Date.now() - (24 - i) * 60_000) },
      firedAt: epochMs(Date.now() - (24 - i) * 60_000),
      outcome: { kind: "ok", text: "Calibrated." },
    })),
    nextDue: null,
    upcoming: [],
  };
  const f = fixture([GENERAL_PROFILE], { jobs: [job] });
  const menu = f.open();
  t.after(() => menu.dispose());
  presses(menu, keys.right, 3);
  type(menu, "calibration");
  const before = text(menu);
  assert.match(before, /› daily-camera/);
  assert(!before.includes("Run now"));
  menu.handleInput(keys.enter);
  assert.match(text(menu), /Jobs › daily-camera-calibration/);
  assert.match(text(menu), /› View history\n\s+Run now\n\s+Delete/);
  menu.handleInput(keys.enter);
  await tick();
  const history = text(menu);
  assert.match(history, /Jobs › daily-camera-calibration runs/);
  menu.handleInput(keys.down);
  assert.notEqual(text(menu), history);
  menu.handleInput(keys.escape);
  assert.match(text(menu), /› View history/);
  menu.handleInput(keys.down);
  menu.handleInput(keys.enter);
  await tick();
  assert.equal(f.calls.at(-1), "run daily-camera-calibration");
  menu.handleInput(keys.down);
  menu.handleInput(keys.enter);
  await tick();
  assert.equal(f.calls.at(-1), "delete daily-camera-calibration");
  menu.handleInput(keys.escape);
  assert.match(text(menu), /Search: calibration/);
  assert.match(text(menu), /› daily-camera/);
});
