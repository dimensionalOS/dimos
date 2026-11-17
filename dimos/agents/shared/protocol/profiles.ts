import type { Static } from "typebox";
import { Type } from "typebox";
import { Value } from "typebox/value";

const fields = {
  name: Type.String({ minLength: 1 }),
  description: Type.String(),
  role: Type.String(),
  blueprints: Type.Array(Type.String({ minLength: 1 })),
  availableSkills: Type.Array(Type.String({ minLength: 1 })),
  alwaysLoadedSkills: Type.Array(Type.String({ minLength: 1 })),
};

export const ProfileSchema = Type.Object(
  { id: Type.String({ minLength: 1 }), ...fields },
  { additionalProperties: false },
);
export const ProfileDraftSchema = Type.Object(
  {
    id: Type.Optional(
      Type.String({
        minLength: 1,
        description: "Internal saved id, only when editing an existing profile. Omit for a new profile.",
      }),
    ),
    ...fields,
  },
  { additionalProperties: false },
);

export type Profile = Static<typeof ProfileSchema>;
export type ProfileDraft = Static<typeof ProfileDraftSchema>;
export type ProfileIdentity = Pick<Profile, "id" | "name" | "description">;
export type ProfileCatalog = { readonly profiles: readonly Profile[]; readonly defaultProfile: string | null };
export type ProfileOptions = {
  readonly blueprints: readonly string[];
  readonly skills: readonly { readonly name: string; readonly description: string }[];
  readonly blueprintsError: string | null;
};

export function parseProfileDraft(value: unknown): ProfileDraft | null {
  return Value.Check(ProfileDraftSchema, value) && value.name.trim() !== "" ? value : null;
}

export const GENERAL_PROFILE: Profile = {
  id: "general",
  name: "General agent",
  description: "DimOS assistance without a robot-specific role.",
  role: "",
  blueprints: [],
  alwaysLoadedSkills: [],
  availableSkills: [],
};

export function profileIdentity(profile: Profile): ProfileIdentity {
  return { id: profile.id, name: profile.name, description: profile.description };
}

export function findProfile(profiles: readonly Profile[], name: string): Profile | undefined {
  return (
    profiles.find((profile) => profile.id === name) ??
    profiles.find((profile) => profile.name.toLowerCase() === name.toLowerCase())
  );
}
