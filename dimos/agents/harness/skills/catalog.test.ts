import assert from "node:assert/strict";
import { mkdir, mkdtemp, rm, writeFile } from "node:fs/promises";
import { join } from "node:path";
import { tmpdir } from "node:os";
import { test } from "node:test";
import { loadSkills, resolveSkill } from "#dimos/agents/harness/skills/catalog.ts";

test("explicit references use the resolved catalog, including user overrides and disabled skills", async (t) => {
  const root = await mkdtemp(join(tmpdir(), "dimos-skills-"));
  t.after(() => rm(root, { recursive: true, force: true }));
  const bundled = join(root, "bundled");
  const user = join(root, "user");
  for (const [base, name, extra] of [
    [bundled, "guide", ""],
    [user, "guide", ""],
    [bundled, "hidden", ""],
    [user, "hidden", "disable-model-invocation: true\n"],
  ] as const) {
    const dir = join(base, name);
    await mkdir(dir, { recursive: true });
    await writeFile(join(dir, "SKILL.md"), `---\nname: ${name}\ndescription: A guide\n${extra}---\nInstructions.`);
  }
  const reports: unknown[] = [];
  const skills = await loadSkills([bundled, user], (error) => reports.push(error));
  assert.deepEqual(skills, [{ name: "guide", description: "A guide", path: join(user, "guide", "SKILL.md") }]);
  for (const text of ["/skill:guide", "/skill:guide request", "/skill:guide\nmultiline request"]) {
    assert.deepEqual(await resolveSkill(text, skills), {
      kind: "loaded",
      skill: {
        name: "guide",
        path: join(user, "guide", "SKILL.md"),
        content: "---\nname: guide\ndescription: A guide\n---\nInstructions.",
      },
    });
  }
  for (const text of ["ordinary", "/help", "/guide", "mention /skill:guide"])
    assert.deepEqual(await resolveSkill(text, skills), { kind: "none" });
  for (const name of ["hidden", "missing", ""])
    assert.deepEqual(await resolveSkill(`/skill:${name}`, skills), {
      kind: "invalid",
      problem: `Unknown skill ${JSON.stringify(name)}.`,
    });
  await rm(join(user, "guide", "SKILL.md"));
  const failed = await resolveSkill("/skill:guide", skills);
  assert.ok(failed.kind === "invalid");
  assert.ok(failed.problem.includes(join(user, "guide", "SKILL.md")));
  assert.deepEqual(reports, []);
});
