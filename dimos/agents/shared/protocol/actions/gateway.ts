import type { ActionGroup } from "./base.ts";
import type { ConversationId, JsonObject } from "@earendil-works/pi-durable";
import type { DialogId } from "../auth.ts";
import type { JobOutcome, JobRow } from "../jobs/state.ts";
import type { ModuleOutcome } from "../runtime/module.ts";
import type { Settings } from "../settings.ts";
import {
  ActionRequestIdSchema,
  AgentModuleNameSchema,
  ConversationIdSchema,
  JobNameSchema,
  ModelRefSchema,
  ThinkingLevelSchema,
} from "../args.ts";
import { Type } from "typebox";
import { action } from "./lib/action.ts";
import { carried, nullable } from "../schema.ts";
import { rules } from "./standing.ts";

const JobActionSchema = Type.Union([
  Type.Object({ kind: Type.Literal("notify"), text: Type.String({ minLength: 1 }) }),
  Type.Object({
    kind: Type.Literal("skill"),
    skill: Type.String({ minLength: 1 }),
    arguments: Type.Unsafe<JsonObject>(Type.Record(Type.String(), Type.Unknown())),
  }),
]);

const JobDraftSchema = Type.Object({
  name: JobNameSchema,
  /** The conversation the job notifies and whose task runs it. */
  conversation: ConversationIdSchema,
  cron: Type.String({ minLength: 1 }),
  /** IANA name; null is the gateway's zone. */
  timeZone: nullable(Type.String()),
  action: JobActionSchema,
});

/** Operating the system: everything that is not one conversation's input or history. */
export const GATEWAY_ACTIONS = {
  createConversation: action({
    args: Type.Object({ request: ActionRequestIdSchema, cwd: nullable(Type.String()) }),
    value: carried<{ readonly conversation: ConversationId }>(),
    executor: "pi-durable",
    access: "owner",
    retry: "by_request_id",
    rule: rules.open,
  }),
  login: action({
    args: Type.Object({
      provider: Type.String(),
      method: Type.Union([Type.Literal("api_key"), Type.Literal("oauth")]),
    }),
    value: carried<null>(),
    executor: "model-runtime",
    access: "owner",
    retry: "unknown",
    rule: rules.open,
  }),
  /** Answers this principal's open prompt; a dialog of another principal is `not_found`. */
  answerLogin: action({
    args: Type.Object({ dialog: Type.Unsafe<DialogId>(Type.String()), value: nullable(Type.String()) }),
    value: carried<null>(),
    executor: "model-runtime",
    access: "owner",
    retry: "unknown",
    rule: rules.open,
  }),
  logout: action({
    args: Type.Object({ provider: Type.String() }),
    value: carried<null>(),
    executor: "model-runtime",
    access: "owner",
    retry: "unknown",
    rule: rules.open,
  }),
  updateSettings: action({
    args: Type.Object({
      model: Type.Optional(nullable(ModelRefSchema)),
      thinkingLevel: Type.Optional(ThinkingLevelSchema),
      theme: Type.Optional(nullable(Type.String({ minLength: 1 }))),
      workspace: Type.Optional(nullable(Type.String({ minLength: 1 }))),
    }),
    value: carried<Settings>(),
    executor: "gateway",
    access: "owner",
    retry: "unknown",
    rule: rules.open,
  }),
  createModule: action({
    args: Type.Object({
      name: AgentModuleNameSchema,
      path: Type.String({ minLength: 1 }),
      /** Sent to HarnessRuntime as `_meta["dimos/conversation"]`, which it records as the module's owner. */
      owner: nullable(ConversationIdSchema),
    }),
    value: carried<ModuleOutcome>(),
    executor: "harness-runtime",
    access: "owner",
    retry: "unknown",
    rule: rules.dimosUp,
  }),
  restartModule: action({
    args: Type.Object({ name: AgentModuleNameSchema }),
    value: carried<ModuleOutcome>(),
    executor: "harness-runtime",
    access: "owner",
    retry: "unknown",
    rule: rules.dimosUp,
  }),
  unloadModule: action({
    args: Type.Object({ name: AgentModuleNameSchema }),
    value: carried<ModuleOutcome>(),
    executor: "harness-runtime",
    access: "owner",
    retry: "unknown",
    rule: rules.dimosUp,
  }),
  deleteModule: action({
    args: Type.Object({ name: AgentModuleNameSchema }),
    value: carried<ModuleOutcome>(),
    executor: "harness-runtime",
    access: "owner",
    retry: "unknown",
    rule: rules.dimosUp,
  }),
  createJob: action({
    args: JobDraftSchema,
    value: carried<JobRow>(),
    executor: "pi-durable",
    access: "owner",
    retry: "unknown",
    rule: rules.open,
  }),
  updateJob: action({
    args: JobDraftSchema,
    value: carried<JobRow>(),
    executor: "pi-durable",
    access: "owner",
    retry: "unknown",
    rule: rules.open,
  }),
  deleteJob: action({
    args: Type.Object({ name: JobNameSchema }),
    value: carried<null>(),
    executor: "pi-durable",
    access: "owner",
    retry: "unknown",
    rule: rules.open,
  }),
  runJob: action({
    args: Type.Object({ name: JobNameSchema, request: ActionRequestIdSchema }),
    value: carried<JobOutcome>(),
    executor: "pi-durable",
    access: "owner",
    retry: "by_request_id",
    rule: rules.open,
  }),
} as const satisfies ActionGroup;
