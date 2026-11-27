import type { Extension } from "@earendil-works/pi-durable";
import { defineExtension, section } from "@earendil-works/pi-durable";
import type { LoadedSkill, SkillSummary } from "#dimos/agents/shared/protocol/skills.ts";
import { dirname, join } from "node:path";
import { parseFrontmatter } from "@earendil-works/pi-coding-agent";
import { readFile, readdir } from "node:fs/promises";

export type SkillInfo = SkillSummary & {
  /** Absolute path of the SKILL.md; loaded by the harness for explicit invocations. */
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

export type SkillResolution =
  | { readonly kind: "none" }
  | { readonly kind: "loaded"; readonly skill: LoadedSkill }
  | { readonly kind: "invalid"; readonly problem: string };

/** Resolve and snapshot a leading explicit skill reference before the input can start a model turn. */
export async function resolveSkill(text: string, skills: readonly SkillInfo[]): Promise<SkillResolution> {
  if (!text.startsWith("/skill:")) return { kind: "none" };
  const name = text.slice("/skill:".length).split(/\s/, 1)[0];
  const skill = skills.find((candidate) => candidate.name === name);
  if (skill === undefined) return { kind: "invalid", problem: `Unknown skill ${JSON.stringify(name)}.` };
  let content: string;
  try {
    content = await readFile(skill.path, "utf8");
  } catch (error) {
    return {
      kind: "invalid",
      problem: `Cannot load skill ${JSON.stringify(name)} from ${skill.path}: ${String(error)}`,
    };
  }
  return { kind: "loaded", skill: { name: skill.name, path: skill.path, content } };
}

/** Model-only instructions accompany the original command, outside its input text limit. */
export function loadedSkillMessage(skill: LoadedSkill): string {
  return [
    `The user explicitly selected skill ${JSON.stringify(skill.name)}. Its full instructions are already loaded below. Follow them for the user's request.`,
    `Skill file: ${JSON.stringify(skill.path)}`,
    `Base directory for relative script, reference, and asset paths: ${JSON.stringify(dirname(skill.path))}`,
    skill.content,
  ].join("\n\n");
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
              "Unless its full instructions are already supplied, read a skill's file before a task it covers:",
              ...skills.map((skill) => `- ${skill.name}: ${skill.description} (${skill.path})`),
            ].join("\n"),
      ),
    ],
  });
}
