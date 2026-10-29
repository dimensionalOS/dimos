import type { Context } from "@earendil-works/chord/context";
import type { ConversationId, Harness, TaskId } from "@earendil-works/pi-durable";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type {
  JobAction,
  JobDefinition,
  JobName,
  JobOutcome,
  JobRun,
  TimeZone,
} from "#dimos/agents/shared/protocol/jobs/state.ts";
import type { JobFiringDeps } from "#dimos/agents/harness/jobs/fire.ts";
import type { JobTask } from "#dimos/agents/harness/jobs/task.ts";
import { CronExpr, localTimeZone, parseTimeZone } from "#dimos/agents/harness/jobs/cron.ts";
import { Type } from "typebox";
import { Value } from "typebox/value";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { fireJob } from "#dimos/agents/harness/jobs/fire.ts";
import { jobIn, putJob, recordRun, removeJob, replaceRun, sameRun } from "#dimos/agents/harness/jobs/registry.ts";
import { withoutAbortSignal } from "@earendil-works/chord/context";

/** A job as a caller states it; the time zone is the gateway's when null. */
export type JobDraft = {
  readonly name: JobName;
  readonly conversation: ConversationId;
  readonly cron: string;
  readonly timeZone: string | null;
  readonly action: JobAction;
};

export type JobChange =
  | { readonly kind: "done"; readonly job: JobDefinition; readonly firstDue: EpochMs }
  | { readonly kind: "exists" }
  | { readonly kind: "missing" }
  | { readonly kind: "invalid"; readonly field: "cron" | "timeZone"; readonly problem: string };

/** The due time a job task keeps in its checkpoint. */
const Checkpoint = Type.Object({ due: Type.Number() });

function problemOf(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

/**
 * The job operations, shared by the model's `job` tool and the gateway's methods. Create and update write the
 * definition and create its task in one commit and abort the previous task; remove takes the definition out and
 * aborts its task; run fires once inside the call and records the run.
 */
export class JobOps {
  private readonly harness: Harness;
  private readonly task: JobTask;
  private readonly firing: JobFiringDeps;

  constructor(harness: Harness, task: JobTask, firing: JobFiringDeps) {
    this.harness = harness;
    this.task = task;
    this.firing = firing;
  }

  async create(draft: JobDraft, context: Context): Promise<JobChange> {
    return this.put("create", draft, context);
  }

  async update(draft: JobDraft, context: Context): Promise<JobChange> {
    return this.put("update", draft, context);
  }

  async remove(name: JobName, context: Context): Promise<JobDefinition | null> {
    const removed = await this.harness.commit((tx) => removeJob(tx, name), context);
    if (removed !== null) await this.harness.abortTask(removed.task, context).catch(() => undefined);
    return removed;
  }

  /**
   * Fires now and records the run; a run recorded already, such as a retried `requested` run, returns that outcome
   * instead. The run is recorded `interrupted` before it fires and its outcome replaces that after, on a context the
   * caller cannot abort, so neither a dropped link nor a restart in between lets the run fire twice.
   */
  async run(name: JobName, run: JobRun, context: Context): Promise<JobOutcome | null> {
    const kept = withoutAbortSignal(context);
    const start = await this.harness.commit(async (tx) => {
      const job = await jobIn(tx, name);
      if (job === null) return null;
      const earlier = job.runs.find((record) => sameRun(record.run, run));
      if (earlier !== undefined) return { kind: "recorded", outcome: earlier.outcome } as const;
      await recordRun(tx, name, job.task, { run, firedAt: epochMs(Date.now()), outcome: { kind: "interrupted" } });
      return { kind: "fire", job } as const;
    }, kept);
    if (start === null) return null;
    if (start.kind === "recorded") return start.outcome;
    const outcome = await fireJob(start.job, run, this.firing, kept);
    await this.harness.commit((tx) => replaceRun(tx, name, { run, firedAt: epochMs(Date.now()), outcome }), kept);
    return outcome;
  }

  /** The due time the job's task sleeps to; null once its task ended. */
  async nextDue(job: JobDefinition, context: Context): Promise<EpochMs | null> {
    const task = await this.harness.getTask(job.task, context);
    const checkpoint = task?.state.checkpoint;
    return Value.Check(Checkpoint, checkpoint) ? epochMs(checkpoint.due) : null;
  }

  private async put(op: "create" | "update", draft: JobDraft, context: Context): Promise<JobChange> {
    let cron: CronExpr;
    try {
      cron = CronExpr.parse(draft.cron);
    } catch (error) {
      return { kind: "invalid", field: "cron", problem: problemOf(error) };
    }
    let timeZone: TimeZone;
    try {
      timeZone = draft.timeZone === null ? localTimeZone() : parseTimeZone(draft.timeZone);
    } catch (error) {
      return { kind: "invalid", field: "timeZone", problem: problemOf(error) };
    }
    let firstDue: EpochMs;
    try {
      firstDue = cron.nextAfter(epochMs(Date.now()), timeZone);
    } catch (error) {
      return { kind: "invalid", field: "cron", problem: problemOf(error) };
    }
    const written = await this.harness.commit(async (tx) => {
      const previous = await jobIn(tx, draft.name);
      if (op === "create" && previous !== null) return { kind: "exists" } as const;
      if (op === "update" && previous === null) return { kind: "missing" } as const;
      const task: TaskId = await tx.createTask(
        this.task,
        { name: draft.name, firstDue },
        { ownership: { kind: "conversation" }, conversationId: draft.conversation, background: true },
      );
      const definition = {
        name: draft.name,
        cron: cron.expression,
        timeZone,
        action: draft.action,
        conversation: draft.conversation,
        task,
        createdAt: previous === null ? epochMs(Date.now()) : previous.createdAt,
      };
      await putJob(tx, definition);
      const job: JobDefinition = { ...definition, runs: previous === null ? [] : previous.runs };
      return { kind: "written", job, previous } as const;
    }, context);
    if (written.kind !== "written") return written;
    if (written.previous !== null) await this.harness.abortTask(written.previous.task, context).catch(() => undefined);
    return { kind: "done", job: written.job, firstDue };
  }
}
