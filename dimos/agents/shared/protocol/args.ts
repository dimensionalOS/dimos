import type { ActionRequestId } from "./actions/request.ts";
import type { AgentModuleName } from "./runtime/event.ts";
import type { CallId } from "./conversation/calls.ts";
import type { ConversationId, EntryId, ModelRef } from "@earendil-works/pi-durable";
import type { JobName } from "./jobs/state.ts";
import { ModelThinkingLevelSchema } from "@earendil-works/pi-ai/providers/model-schema";
import { Type } from "typebox";
import { nullable } from "./schema.ts";

export const ActionRequestIdSchema = Type.Unsafe<ActionRequestId>(Type.String({ format: "uuid" }));

export const ConversationIdSchema = Type.Unsafe<ConversationId>(Type.Integer({ minimum: 1 }));

export const EntryIdSchema = Type.Unsafe<EntryId>(Type.Integer({ minimum: 1 }));

export const CallIdSchema = Type.Unsafe<CallId>(Type.String({ minLength: 1 }));

export const ModelRefSchema = Type.Unsafe<ModelRef>(Type.Object({ provider: Type.String(), modelId: Type.String() }));

export const ThinkingLevelSchema = nullable(ModelThinkingLevelSchema);

export const AgentModuleNameSchema = Type.Unsafe<AgentModuleName>(Type.String({ pattern: "^[A-Za-z_][A-Za-z0-9_]*$" }));

export const JobNameSchema = Type.Unsafe<JobName>(Type.String({ pattern: "^[a-z0-9_-]{1,64}$" }));
