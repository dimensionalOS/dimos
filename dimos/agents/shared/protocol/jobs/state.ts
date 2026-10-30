import type { ConversationId, JsonObject, TaskId } from "@earendil-works/pi-durable";
import type { EpochMs } from "../time.ts";

/** An IANA zone name that `Intl.DateTimeFormat` accepts. */
export type TimeZone = string & { readonly __brand: "TimeZone" };

/** Lowercase letters, digits, "-" and "_", at most 64 characters. */
export type JobName = string & { readonly __brand: "JobName" };

export type JobAction =
  /** Starts a turn in the job's conversation. */
  | { readonly kind: "notify"; readonly text: string }
  /** Calls any `@skill` through McpServer, without a turn. */
  | { readonly kind: "skill"; readonly skill: string; readonly arguments: JsonObject };

export type JobRun =
  | { readonly kind: "scheduled"; readonly due: EpochMs }
  /** `job op=run`, inside the tool call that asked for it. */
  | { readonly kind: "manual"; readonly toolCall: string }
  /** "run now" from a frontend; the request id makes a retried run fire once. */
  | { readonly kind: "requested"; readonly request: string };

export type JobOutcome =
  | { readonly kind: "submitted" }
  | { readonly kind: "ok"; readonly text: string }
  | { readonly kind: "error"; readonly text: string }
  /** A skill call that a crash interrupted; it is never called twice. */
  | { readonly kind: "interrupted" };

export type JobRunRecord = {
  readonly run: JobRun;
  readonly firedAt: EpochMs;
  readonly outcome: JobOutcome;
};

export type JobDefinition = {
  readonly name: JobName;
  readonly cron: string;
  readonly timeZone: TimeZone;
  readonly action: JobAction;
  /** The conversation that created the job: its task lives there and its notify runs start turns there. */
  readonly conversation: ConversationId;
  /** The job's live durable task. A task that wakes and finds another id here ends without running. */
  readonly task: TaskId;
  readonly createdAt: EpochMs;
  /** Newest last, at most 20. */
  readonly runs: JobRunRecord[];
};

/** A job as a user sees it: its definition without its durable task, its next due time and the due times after. */
export type JobRow = Omit<JobDefinition, "task"> & {
  /** The job task's checkpoint; null once its task ended. */
  readonly nextDue: EpochMs | null;
  /** The next three due times after `nextDue`, from the cron expression, for the schedule view. */
  readonly upcoming: readonly EpochMs[];
};
