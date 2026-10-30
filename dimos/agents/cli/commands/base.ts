import type { ConversationId } from "@earendil-works/pi-durable";
import type { AgentPaths } from "../../gateway/config/paths.ts";

export type CommandContext = {
  readonly paths: AgentPaths;
  readonly env: NodeJS.ProcessEnv;
  readonly conversation: ConversationId | null;
  readonly cwd: string | null;
  readonly view: boolean;
  readonly json: boolean;
  readonly daemon: boolean;
  readonly args: readonly string[];
  readonly bundledSkillsDir: string;
  readonly startGateway: () => Promise<void>;
};

export type Command = (context: CommandContext) => number | Promise<number>;
