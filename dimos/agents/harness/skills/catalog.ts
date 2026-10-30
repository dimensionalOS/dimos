import type { Extension } from "@earendil-works/pi-durable";
import { defineExtension, section } from "@earendil-works/pi-durable";
import { join } from "node:path";
import { parseFrontmatter } from "@earendil-works/pi-coding-agent";
import { readFile, readdir } from "node:fs/promises";

type SkillInfo = {
  /** From the SKILL.md frontmatter. */
  readonly name: string;
  /** From the SKILL.md frontmatter, on one line. */
  readonly description: string;
  /** Absolute path of the SKILL.md; the model loads it with `read`. */
  readonly path: string;
};

/**
 * Every `*\/SKILL.md` under the roots, in root order; a later root's skill replaces an earlier one with the same name,
 * and one with `disable-model-invocation: true` takes it off the list. A missing root is empty; a SKILL.md whose YAML
 * frontmatter does not parse, or has no name and description, is reported and skipped.
 */
export async function loadSkills(
  roots: readonly string[],
  report: (error: unknown) => void,
): Promise<readonly SkillInfo[]> {
  const skills = new Map<string, SkillInfo>();
  for (const root of roots) {
    const entries = await readdir(root, { withFileTypes: true }).catch(() => []);
    for (const entry of entries) {
      if (!entry.isDirectory()) continue;
      const path = join(root, entry.name, "SKILL.md");
      const text = await readFile(path, "utf8").catch(() => null);
      if (text === null) continue;
      let frontmatter: Record<string, unknown>;
      try {
        frontmatter = parseFrontmatter(text).frontmatter;
      } catch (error) {
        report(new Error(`${path}: ${String(error)}`));
        continue;
      }
      const { name, description } = frontmatter;
      if (typeof name !== "string" || name === "" || typeof description !== "string" || description.trim() === "") {
        report(new Error(`${path}: no name and description in its frontmatter`));
        continue;
      }
      if (frontmatter["disable-model-invocation"] === true) skills.delete(name);
      else skills.set(name, { name, description: description.replace(/\s+/g, " ").trim(), path });
    }
  }
  return [...skills.values()];
}

/**
 * The `skills` section: one line per skill with its path. The agent loads a skill by reading that file, so skills
 * need no tool of their own and the tool list stays exactly the plan's table.
 */
export function skillsExtension(skills: readonly SkillInfo[]): Extension {
  return defineExtension({
    name: "dimos-skills",
    sections: [
      section("skills", () =>
        skills.length === 0
          ? undefined
          : [
              "Read a skill's file before a task it covers:",
              ...skills.map((skill) => `- ${skill.name}: ${skill.description} (${skill.path})`),
            ].join("\n"),
      ),
    ],
  });
}
