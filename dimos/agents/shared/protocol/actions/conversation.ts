import type { ActionGroup } from "./base.ts";
import type { ConversationId, SubmissionId, TaskId } from "@earendil-works/pi-durable";
import type { ImageContent } from "@earendil-works/pi-ai";
import type { QueuedText } from "../conversation/queue.ts";
import type { UserRequestId } from "../conversation/input.ts";
import { ActionRequestIdSchema, CallIdSchema, EntryIdSchema, ModelRefSchema, ThinkingLevelSchema } from "../args.ts";
import { ImageAddressSchema } from "../conversation/images.ts";
import { Type } from "typebox";
import { action } from "./lib/action.ts";
import { carried, nullable } from "../schema.ts";
import { rules } from "./standing.ts";

const ImageSchema = Type.Unsafe<ImageContent>(
  Type.Object({ type: Type.Literal("image"), data: Type.String(), mimeType: Type.String() }),
);

const UserRequestIdSchema = Type.Unsafe<UserRequestId>(Type.String({ format: "uuid" }));

const message = EntryIdSchema;

/** A conversation's input and history, plus the actions every attached client has. */
export const CONVERSATION_ACTIONS = {
  send: action({
    args: Type.Object({
      request: UserRequestIdSchema,
      text: Type.String(),
      images: Type.Array(ImageSchema),
      /** Only matters while a run is in progress: a steer joins the run, a follow-up waits for its end. */
      whenBusy: Type.Union([Type.Literal("steer"), Type.Literal("followUp")]),
    }),
    value: carried<{ readonly submission: SubmissionId }>(),
    executor: "pi-durable",
    access: "owner",
    retry: "by_request_id",
    rule: rules.open,
  }),
  withdraw: action({
    args: Type.Object({ request: ActionRequestIdSchema }),
    value: carried<readonly QueuedText[]>(),
    executor: "pi-durable",
    access: "owner",
    retry: "by_request_id",
    rule: rules.queued,
  }),
  abort: action({
    args: Type.Object({ request: ActionRequestIdSchema }),
    value: carried<readonly QueuedText[]>(),
    executor: "pi-durable",
    access: "owner",
    retry: "by_request_id",
    rule: rules.open,
  }),
  configure: action({
    args: Type.Object({ model: nullable(ModelRefSchema), thinkingLevel: ThinkingLevelSchema }),
    value: carried<null>(),
    executor: "pi-durable",
    access: "owner",
    retry: "unknown",
    rule: rules.idle,
  }),
  /** The edited text becomes a user input in the fork whose request is the edit's own, so a retry places it once. */
  edit: action({
    args: Type.Object({
      request: UserRequestIdSchema,
      message,
      text: Type.String(),
      images: Type.Array(ImageSchema),
    }),
    value: carried<{ readonly conversation: ConversationId; readonly kind: "fork" | "new" }>(),
    executor: "pi-durable",
    access: "owner",
    retry: "by_request_id",
    rule: rules.userMessage,
  }),
  summarizeUpTo: action({
    args: Type.Object({ request: ActionRequestIdSchema, message }),
    value: carried<{ readonly conversation: ConversationId }>(),
    executor: "model-runtime",
    access: "owner",
    retry: "by_request_id",
    rule: rules.inRange,
  }),
  summarizeFrom: action({
    args: Type.Object({ request: ActionRequestIdSchema, message }),
    value: carried<{ readonly conversation: ConversationId }>(),
    executor: "model-runtime",
    access: "owner",
    retry: "by_request_id",
    rule: rules.inRange,
  }),
  background: action({
    args: Type.Object({ call: CallIdSchema }),
    value: carried<{ readonly task: TaskId }>(),
    executor: "gateway",
    access: "owner",
    retry: "unknown",
    rule: rules.detachable,
  }),
  /** Ends a moved call's work; the model gets its output so far as a `stopped` notice. */
  stopBackground: action({
    args: Type.Object({ call: CallIdSchema }),
    value: carried<null>(),
    executor: "gateway",
    access: "owner",
    retry: "unknown",
    rule: rules.moved,
  }),
  /** The bytes of one image the transcript carries only as a ref, in its stored format. */
  image: action({
    args: Type.Object({ at: ImageAddressSchema }),
    value: carried<ImageContent>(),
    executor: "pi-durable",
    access: "any",
    retry: "unknown",
    rule: rules.open,
  }),
} as const satisfies ActionGroup;
