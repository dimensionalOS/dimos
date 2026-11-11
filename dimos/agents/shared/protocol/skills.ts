export type SkillSummary = {
  readonly name: string;
  readonly description: string;
};

/** The selected file as loaded when its input was accepted. */
export type LoadedSkill = {
  readonly name: string;
  readonly path: string;
  readonly content: string;
};
