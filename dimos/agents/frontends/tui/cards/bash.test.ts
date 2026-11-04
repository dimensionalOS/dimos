import assert from "node:assert/strict";
import type { ToolResult } from "#dimos/agents/shared/protocol/conversation/state.ts";
import { BashCard } from "#dimos/agents/frontends/tui/cards/bash.ts";
import { ProcessTerminal, TuiMainScreen, visibleWidth } from "@earendil-works/pi-tui";
import { initTheme } from "@earendil-works/pi-coding-agent";
import { once } from "node:events";
import { setTimeout as delay } from "node:timers/promises";
import { spawn } from "node:child_process";
import { stripVTControlCharacters } from "node:util";
import { test } from "node:test";

test("shell card streams real output, bounds running rows, collapses on exit and disposes its timer", async (t) => {
  initTheme("dark", false);
  const tui = new TuiMainScreen(new ProcessTerminal());
  let renders = 0;
  tui.requestRender = () => {
    renders++;
  };
  const card = new BashCard(
    { type: "toolCall", id: "call-1", name: "bash", arguments: { command: "node install-fixture.js" } },
    tui,
    process.cwd(),
  );
  t.after(() => card.dispose());
  assert.match(stripVTControlCharacters(card.render(90).join("\n")), /Waiting for command output/);
  let output = "";
  const child = spawn(process.execPath, [
    "-e",
    `for(let i=0;i<16;i++) console.log('+ package-'+i); setTimeout(()=>console.log('Installed 16 packages'),100)`,
  ]);
  t.after(() => child.kill());
  child.stdout.on("data", (data) => {
    output += data;
    card.output(output);
  });
  await once(child, "exit");
  const running = stripVTControlCharacters(card.render(90).join("\n"));
  assert.match(running, /package-15/);
  assert.match(running, /Installed 16 packages/);
  assert.equal(running.match(/Installed 16 packages/g)?.length, 1);
  assert.match(running, /Running · /);
  assert(!running.includes("package-0\n"));
  assert(card.render(90).length < 20);
  const before = renders;
  await delay(120);
  assert(renders > before, "the loader redraws while the command runs");
  const result = {
    role: "toolResult",
    toolCallId: "call-1",
    toolName: "bash",
    content: [{ type: "text", text: output }],
    details: {
      tool: "bash",
      details: { truncation: null, fullOutputPath: null, exitCode: 0, render: null, limit: null },
    },
    isError: false,
    timestamp: 0,
  } satisfies ToolResult;
  card.end(result);
  const completed = stripVTControlCharacters(card.render(90).join("\n"));
  assert.match(completed, /Done · \d+\.\ds/);
  assert.equal(completed.match(/Installed 16 packages/g)?.length, 1);
  assert(card.render(90).length < 9);
  for (const width of [30, 90]) assert(card.render(width).every((line) => visibleWidth(line) <= width));
  card.end({ ...result, content: [{ type: "text", text: `${output}\nfirst\nsecond\nthird\nfourth` }] });
  assert.match(stripVTControlCharacters(card.render(90).join("\n")), /Done · Installed 16 packages/);
  card.dispose();
  const disposed = renders;
  await delay(120);
  assert.equal(renders, disposed, "no redraws after disposal");
});
