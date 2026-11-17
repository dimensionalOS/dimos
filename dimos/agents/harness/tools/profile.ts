import type { AgentTool } from "#dimos/agents/harness/tools/base.ts";
import type { Profiles } from "#dimos/agents/harness/profiles.ts";
import { ProfileDraftSchema, findProfile } from "#dimos/agents/shared/protocol/profiles.ts";
import { ProfileError, boundProfile, profileCatalog } from "#dimos/agents/harness/profiles.ts";
import { Type } from "typebox";
import { agentTool } from "#dimos/agents/harness/tools/lib/tool.ts";
import { defineTool } from "@earendil-works/pi-durable";

export function profileTool(profiles: Profiles): AgentTool<"profile"> {
  const tool = defineTool({
    name: "profile",
    description:
      "List or get reusable agent profiles, save a profile when asked, or read an available skill's frozen instructions for this conversation. A profile has a name, description, role, blueprints, available skills, and always-loaded skills. Omit id when creating; keep the returned internal id when editing. Saving updates the catalog immediately, without starting DimOS or changing this conversation's profile.",
    parameters: Type.Object({
      op: Type.Union([Type.Literal("list"), Type.Literal("get"), Type.Literal("save"), Type.Literal("read_skill")]),
      name: Type.Optional(Type.String({ description: "Profile name for get; skill name for read_skill." })),
      profile: Type.Optional(ProfileDraftSchema),
    }),
    execute: async (args, api, context) => {
      try {
        let result: unknown;
        switch (args.op) {
          case "list":
            result = await profileCatalog(api, context);
            break;
          case "get": {
            const catalog = await profileCatalog(api, context);
            result = findProfile(catalog.profiles, args.name ?? "");
            if (result === undefined) throw new ProfileError(`Unknown profile ${args.name ?? ""}.`);
            break;
          }
          case "save":
            if (args.profile === undefined) throw new ProfileError("save needs a complete profile definition.");
            result = await profiles.save(args.profile, context);
            break;
          case "read_skill": {
            const bound = await boundProfile(api, api.conversationId, context);
            const skill = bound.skills.find((skill) => skill.name === args.name);
            if (skill === undefined)
              throw new ProfileError(`No profile skill ${args.name ?? ""} in this conversation.`);
            return { output: [{ type: "text", text: skill.instructions }], details: {} };
          }
        }
        return { output: [{ type: "text", text: JSON.stringify(result) }], details: {} };
      } catch (error) {
        if (!(error instanceof ProfileError)) throw error;
        return { output: [{ type: "text", text: error.message }], isError: true, details: {} };
      }
    },
  });
  return agentTool("profile", tool, {
    replay: "unsafe",
    settle: null,
    fullTextAt: null,
    wholeResult: (call) => call.arguments["op"] === "read_skill",
    background: null,
  });
}
