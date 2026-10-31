import type { Branch } from "../../../../harness/history/forks.ts";
import type { ConversationId, EntryId, SubmissionId } from "@earendil-works/pi-durable";
import type { JobChange } from "../../../../harness/jobs/ops.ts";
import type { JobName, JobRow } from "../../../../shared/protocol/jobs/state.ts";
import { jobRow } from "./jobs.ts";
import type { ModuleOutcome } from "../../../../shared/protocol/runtime/module.ts";
import type { Outcome } from "../../../../shared/protocol/outcome.ts";
import type { SubmitResult } from "../../../../harness/inputs/gate.ts";
import { done, refused } from "../../../../shared/protocol/outcome.ts";

/** `ok` is done with the outcome; otherwise `module_failed` with its one-line error and traceback. */
export function fromModuleOutcome(outcome: ModuleOutcome): Outcome<ModuleOutcome> {
  if (outcome.ok) return done(outcome);
  return refused({
    code: "module_failed",
    module: outcome.name,
    op: outcome.op,
    error: outcome.error === null ? `${outcome.op} ${outcome.name} failed` : outcome.error,
    traceback: outcome.traceback,
  });
}

export function fromJobChange(change: JobChange, name: JobName): Outcome<JobRow> {
  switch (change.kind) {
    case "done":
      return done(jobRow(change.job, change.firstDue));
    case "exists":
      return refused({ code: "invalid", field: "name", problem: `job ${name} exists; update it instead` });
    case "missing":
      return refused({ code: "not_found", what: "job", name });
    case "invalid":
      return refused({ code: "invalid", field: change.field, problem: change.problem });
  }
}

/** `unknown_conversation` is `not_found`; a busy rejection is `running`; a runtime steer with nothing to say never comes from a user. */
export function fromSubmit(
  result: SubmitResult,
  conversation: ConversationId,
): Outcome<{ readonly submission: SubmissionId }> {
  switch (result.kind) {
    case "submitted":
      return done({ submission: result.submission });
    case "unknown_conversation":
      return refused({ code: "not_found", what: "conversation", name: String(conversation) });
    case "busy":
      return refused({ code: "running", conversation });
    case "nothing_to_say":
      return refused({ code: "invalid", field: "text", problem: "there is nothing to send" });
  }
}

export function fromBranch(
  branch: Branch,
  source: ConversationId,
  message: EntryId,
): Outcome<{ readonly conversation: ConversationId; readonly kind: "fork" | "new" }> {
  switch (branch.kind) {
    case "fork":
    case "new":
      return done({ conversation: branch.conversation, kind: branch.kind });
    case "not_found":
      return refused({
        code: "not_found",
        what: branch.what,
        name: branch.what === "conversation" ? String(source) : String(message),
      });
    case "not_a_user_message":
      return refused({ code: "not_a_user_message", conversation: source, entry: message });
    case "nothing_before":
      return refused({ code: "invalid", field: "message", problem: "nothing comes before it to summarize" });
    case "stale":
      return refused({ code: "stale", conversation: source });
    case "model_failed":
      return refused({ code: "model_failed", error: branch.error });
  }
}
