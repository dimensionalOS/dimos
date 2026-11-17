import assert from "node:assert/strict";
import { test } from "node:test";
import { visibleWidth } from "@earendil-works/pi-tui";
import { initTheme } from "@earendil-works/pi-coding-agent";
import { Status } from "#dimos/agents/shared/protocol/status.ts";
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
const keys = { down: "\x1b[B", up: "\x1b[A", right: "\x1b[C", left: "\x1b[D", enter: "\r", escape: "\x1b" };
const tick = () => new Promise((resolve) => setImmediate(resolve));
initTheme("dark", false);

function fixture(
  options: {
    jobs?: readonly JobRow[];
    modules?: readonly ModuleRow[];
    conversations?: readonly ConversationRow[];
  } = {},
) {
  const calls: string[] = [];
  const ended = new Promise<never>(() => {});
  const actions = new Feedback(ended);
  const gateway = {
    variant: "owner",
    link: state({ state: "up" }),
    load: state({ state: "idle" }),
    runs: state([]),
    conversations: state(options.conversations ?? []),
    modules: state(options.modules ?? []),
    jobs: state(options.jobs ?? []),
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
  let attached = 0;
  const memory = newMenuMemory();
  const open = () =>
    new MenuView({
      ui,
      memory,
      current: null,
      attach: (id) => {
        attached = id;
      },
      close: () => {},
      requestRender: () => {},
      height: () => 25,
    });
  return { open, gateway, calls, attached: () => attached };
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
function conversation(id: number): ConversationRow {
  return {
    id: Value.Parse(ConversationIdSchema, id),
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

test("terminal-encoded printable keys filter each section and preserve its query", (t) => {
  const f = fixture();
  const menu = f.open();
  t.after(() => menu.dispose());
  for (let section = 0; section < 3; section++) {
    menu.handleInput("\x1b[45:95;2u");
    menu.handleInput("\x1b[97u");
    assert.match(text(menu), /Search: _a/);
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

test("Conversations Enter resumes immediately", async (t) => {
  const f = fixture({ conversations: [conversation(8)] });
  const menu = f.open();
  t.after(() => menu.dispose());
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
  const f = fixture({ modules: [module] });
  const menu = f.open();
  t.after(() => menu.dispose());
  menu.handleInput(keys.right);
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
  const f = fixture({ jobs: [job] });
  const menu = f.open();
  t.after(() => menu.dispose());
  presses(menu, keys.right, 2);
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
