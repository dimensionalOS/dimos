import type { Context } from "@earendil-works/chord/context";
import type { ConversationId, DocumentReader, Harness, Tx } from "@earendil-works/pi-durable";
import type { Profile, ProfileCatalog, ProfileDraft, ProfileOptions } from "#dimos/agents/shared/protocol/profiles.ts";
import type { DimosInstall } from "#dimos/agents/harness/dimos/install.ts";
import type { SkillInfo } from "#dimos/agents/harness/skills/catalog.ts";
import { GENERAL_PROFILE, findProfile, parseProfileDraft } from "#dimos/agents/shared/protocol/profiles.ts";
import { Type } from "typebox";
import { Value } from "typebox/value";
import { defineDoc, defineExtension, section } from "@earendil-works/pi-durable";
import { loadSkills } from "#dimos/agents/harness/skills/catalog.ts";
import { readFile } from "node:fs/promises";
import { execFile } from "node:child_process";
import { promisify } from "node:util";
import { randomUUID } from "node:crypto";

const runFile = promisify(execFile);
const BLUEPRINT_NAMES = [
  "import json",
  "from dimos.robot.all_blueprints import all_blueprints",
  "from dimos.robot.external_blueprints import list_external_blueprint_names",
  "print(json.dumps(sorted([name for name in all_blueprints if not name.startswith('demo-')] + list_external_blueprint_names())))",
].join("\n");

export const ProfilesDoc = defineDoc<{ profiles: Profile[]; defaultProfile: string | null }>({
  kind: "dimos.profiles",
  version: 1,
  scope: "session",
  initial: () => ({ profiles: [], defaultProfile: null }),
});

type BoundSkill = { name: string; description: string; instructions: string; mode: "always" | "available" };
export type BoundProfile = { profile: Profile; skills: BoundSkill[] };

export const BoundProfileDoc = defineDoc<BoundProfile>({
  kind: "dimos.profile",
  version: 1,
  scope: "conversation",
  history: "latest",
  fork: "current",
  initial: () => ({ profile: structuredClone(GENERAL_PROFILE), skills: [] }),
});

export class ProfileError extends Error {}

export async function profileCatalog(reader: DocumentReader, context: Context): Promise<ProfileCatalog> {
  return catalogFromSaved(await reader.snapshot(ProfilesDoc, context));
}

export function catalogFromSaved(saved: ProfileCatalog | null | undefined): ProfileCatalog {
  return {
    profiles: [GENERAL_PROFILE, ...(saved?.profiles ?? [])].sort((a, b) => a.name.localeCompare(b.name)),
    defaultProfile: saved?.defaultProfile ?? null,
  };
}

export async function boundProfile(
  reader: DocumentReader,
  conversation: ConversationId,
  context: Context,
): Promise<BoundProfile> {
  return (await reader.snapshot(BoundProfileDoc, conversation, context)) ?? { profile: GENERAL_PROFILE, skills: [] };
}

export async function bindProfile(tx: Tx, conversation: ConversationId, bound: BoundProfile): Promise<void> {
  const doc = await tx.doc(BoundProfileDoc, conversation);
  doc.profile = bound.profile;
  doc.skills = bound.skills;
}

/** The menu and agent edit the same catalog; conversations only read their creation-time snapshot. */
export class Profiles {
  private readonly options: {
    readonly harness: Harness;
    readonly skillRoots: readonly string[];
    readonly report: (error: unknown) => void;
    readonly install: Pick<DimosInstall, "python" | "root">;
    readonly env: NodeJS.ProcessEnv;
  };

  constructor(options: Profiles["options"]) {
    this.options = options;
  }

  async save(draft: ProfileDraft, context: Context): Promise<Profile> {
    if (parseProfileDraft(draft) === null) throw new ProfileError("Invalid profile definition.");
    const profile: Profile = { ...draft, id: draft.id ?? randomUUID(), name: draft.name.trim() };
    if (profile.id === GENERAL_PROFILE.id)
      throw new ProfileError("General agent is built in. Create a new profile to customize it.");
    if (profile.availableSkills.some((name) => profile.alwaysLoadedSkills.includes(name)))
      throw new ProfileError("A skill must be available or always loaded, not both.");
    await this.options.harness.commit(async (tx) => {
      const doc = await tx.doc(ProfilesDoc);
      if (
        [GENERAL_PROFILE, ...doc.profiles].some(
          (saved) => saved.id !== profile.id && saved.name.toLowerCase() === profile.name.toLowerCase(),
        )
      )
        throw new ProfileError(`A profile named ${JSON.stringify(profile.name)} already exists.`);
      const index = doc.profiles.findIndex((saved) => saved.id === profile.id);
      if (draft.id === undefined) doc.profiles.push(profile);
      else {
        if (index === -1) throw new ProfileError(`Unknown profile ${JSON.stringify(profile.id)}.`);
        doc.profiles[index] = profile;
      }
    }, context);
    return profile;
  }

  async delete(id: string, context: Context): Promise<void> {
    if (id === GENERAL_PROFILE.id) throw new ProfileError("General agent is built in and cannot be deleted.");
    await this.options.harness.commit(async (tx) => {
      const doc = await tx.doc(ProfilesDoc);
      const index = doc.profiles.findIndex((profile) => profile.id === id);
      if (index === -1) throw new ProfileError(`Unknown profile ${JSON.stringify(id)}.`);
      doc.profiles.splice(index, 1);
      if (doc.defaultProfile === id) doc.defaultProfile = null;
    }, context);
  }

  async setDefault(profile: string | null, context: Context): Promise<void> {
    await this.options.harness.commit(async (tx) => {
      const doc = await tx.doc(ProfilesDoc);
      if (profile !== null && profile !== GENERAL_PROFILE.id && !doc.profiles.some((saved) => saved.id === profile))
        throw new ProfileError(`Unknown profile ${JSON.stringify(profile)}.`);
      doc.defaultProfile = profile;
    }, context);
  }

  async resolve(profile: string | null | undefined, context: Context): Promise<BoundProfile> {
    const catalog = await profileCatalog(this.options.harness, context);
    const id = profile ?? catalog.defaultProfile ?? GENERAL_PROFILE.id;
    const selected = findProfile(catalog.profiles, id);
    if (selected === undefined) throw new ProfileError(`Unknown profile ${JSON.stringify(id)}.`);
    const skills: BoundSkill[] = [];
    if (!selected.availableSkills.length && !selected.alwaysLoadedSkills.length) return { profile: selected, skills };
    const catalogSkills = await loadSkills(this.options.skillRoots, this.options.report);
    const always = new Set(selected.alwaysLoadedSkills);
    for (const name of new Set([...always, ...selected.availableSkills])) {
      const skill = catalogSkills.find((candidate) => candidate.name === name);
      if (skill === undefined) throw new ProfileError(`Profile ${id} requires missing skill ${JSON.stringify(name)}.`);
      let instructions: string;
      try {
        instructions = await readFile(skill.path, "utf8");
      } catch {
        throw new ProfileError(`Cannot read skill ${name} at ${skill.path}.`);
      }
      skills.push({
        name,
        description: skill.description,
        instructions,
        mode: always.has(name) ? "always" : "available",
      });
    }
    return { profile: selected, skills };
  }

  async choices(): Promise<ProfileOptions> {
    const skills = (await loadSkills(this.options.skillRoots, this.options.report)).map(({ name, description }) => ({
      name,
      description,
    }));
    try {
      const { stdout } = await runFile(this.options.install.python, ["-c", BLUEPRINT_NAMES], {
        cwd: this.options.install.root,
        env: this.options.env,
        timeout: 10_000,
        maxBuffer: 1024 * 1024,
      });
      const names: unknown = JSON.parse(stdout);
      if (!Value.Check(Type.Array(Type.String()), names)) throw new Error("Invalid blueprint catalog.");
      return { blueprints: names, skills, blueprintsError: null };
    } catch {
      return {
        blueprints: [],
        skills,
        blueprintsError: "Could not read blueprints from the configured DimOS installation.",
      };
    }
  }
}

export function profileExtension(options: {
  readonly generalSkills: readonly SkillInfo[];
  readonly baseSkills: readonly SkillInfo[];
}) {
  return defineExtension({
    name: "dimos-profile",
    sections: [
      section("skills", async (input, context) => {
        const bound = await boundProfile(input.read, input.conversationId, context);
        const skills = (bound.profile.id === GENERAL_PROFILE.id ? options.generalSkills : options.baseSkills).filter(
          (skill) => !bound.skills.some((selected) => selected.name === skill.name),
        );
        return skills.length === 0
          ? undefined
          : [
              "Read a skill's file before a task it covers:",
              ...skills.map((skill) => `- ${skill.name}: ${skill.description} (${skill.path})`),
            ].join("\n");
      }),
      section("profile", async (input, context) => {
        const { profile, skills } = await boundProfile(input.read, input.conversationId, context);
        return [
          `Profile: ${profile.name} (${profile.id}). ${profile.description}`,
          profile.role,
          `Expected blueprints: ${profile.blueprints.join(", ") || "none specified"}. Check live runtime observations for actual conditions. Loading a profile does not start DimOS.`,
          "Use the profile tool to save a reusable profile only when asked. Profile edits and defaults affect future conversations.",
          ...skills.map((skill) =>
            skill.mode === "always"
              ? `Always-loaded skill ${skill.name}:\n${skill.instructions}`
              : `Available skill ${skill.name}: ${skill.description}. Read its frozen instructions with profile op=read_skill name=${JSON.stringify(skill.name)} when relevant.`,
          ),
        ]
          .filter((line) => line !== "")
          .join("\n\n");
      }),
    ],
  });
}
