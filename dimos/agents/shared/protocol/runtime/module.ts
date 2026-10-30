import type { AgentModuleName } from "./event.ts";
import type { ConversationId } from "@earendil-works/pi-durable";
import type { Static } from "typebox";
import { Type } from "typebox";
import { nullable } from "../schema.ts";

/** `AgentModuleStatus` (`agent_module_manager.py`); inventory's agent_modules rows and `list` share it. */
export const AgentModuleRecordSchema = Type.Object({
  name: Type.Unsafe<AgentModuleName>(Type.String()),
  class: Type.String(),
  state: Type.Union([Type.Literal("running"), Type.Literal("stopped")]),
  /** The revision the running system holds; null when it does not run the module. */
  revision: nullable(Type.Integer()),
  /** The conversation that created it; null for modules made outside any conversation. */
  owner: nullable(Type.Unsafe<ConversationId>(Type.Integer({ minimum: 1 }))),
  last_error: nullable(Type.String()),
  path: Type.String(),
});

export type AgentModuleRecord = Static<typeof AgentModuleRecordSchema>;

export const ModuleOutcomeSchema = Type.Object({
  name: Type.Unsafe<AgentModuleName>(Type.String()),
  op: Type.Union([Type.Literal("create"), Type.Literal("restart"), Type.Literal("unload"), Type.Literal("delete")]),
  ok: Type.Boolean(),
  state: Type.Union([Type.Literal("running"), Type.Literal("stopped"), Type.Literal("absent")]),
  path: nullable(Type.String()),
  revision: nullable(Type.Integer()),
  error: nullable(Type.String()),
  traceback: nullable(Type.String()),
});

/** `AgentModuleOutcome` (`agent_module_manager.py`), never source. */
export type ModuleOutcome = Static<typeof ModuleOutcomeSchema>;

/** The class name an agent module runs under joins it to inventory's runtime instance and through that to its streams. */
export type ModuleInstance = string & { readonly __brand: "ModuleInstance" };

/** One agent module, from HarnessRuntime when the link is up and from its module.json on disk when it is not. */
export type ModuleRow = {
  readonly name: AgentModuleName;
  readonly class: string;
  /** The conversation that created it; null for modules made outside any conversation. */
  readonly owner: ConversationId | null;
  readonly path: string;
  readonly lastError: string | null;
  /** `unknown` while DimOS is unreachable: only the running system knows. */
  readonly state: "running" | "stopped" | "unknown";
  readonly revision: number | null;
  readonly instance: ModuleInstance | null;
};
