import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { JobDefinition, JobName, JobOutcome, JobRun } from "#dimos/agents/shared/protocol/jobs/state.ts";
import type { JobFiringDeps } from "#dimos/agents/harness/jobs/fire.ts";
import type { NextTaskState, Task, TaskId, Tx } from "@earendil-works/pi-durable";
import { CronExpr } from "#dimos/agents/harness/jobs/cron.ts";
import { defineTask } from "@earendil-works/pi-durable";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { fireJob } from "#dimos/agents/harness/jobs/fire.ts";
import { jobIn, recordRun } from "#dimos/agents/harness/jobs/registry.ts";

type JobTaskInput = {
  readonly name: JobName;
  readonly firstDue: EpochMs;
};

/** The due time is kept here; `sleep(until)` keeps none. A skill run commits `attempted` before its call. */
type JobCheckpoint =
  | { readonly phase: "sleeping"; readonly due: EpochMs }
  | { readonly phase: "firing"; readonly due: EpochMs; readonly attempted: boolean };

/** Why a job's task ended: an update gave the job a new task, or the job was deleted. */
type JobTaskResult = { readonly ended: "superseded" | "deleted" };

type NextState = NextTaskState<JobCheckpoint, JobTaskResult>;

export type JobTask = Task<JobTaskInput, JobCheckpoint, JobTaskResult, object>;

/**
 * One cron job as a `background: true` task in its conversation: sleep to `due`, fire once, next due after the later
 * of now and `due`. A notify run repeats harmlessly after a crash; a skill run is never called twice and becomes
 * `interrupted` instead.
 */
export function createJobTask(deps: JobFiringDeps): JobTask {
  return defineTask<JobTaskInput, JobCheckpoint, JobTaskResult>({
    name: "dimos.job",
    version: 1,
    initial: (input) => ({ phase: "sleeping", due: input.firstDue }),
    phases: {
      sleeping: async (task, runtime, context) => {
        const { due } = task.state.checkpoint;
        await runtime.sleep(due, context);
        await runtime.commit(async (tx) => {
          const job = await liveJob(tx, task.input.name, runtime.taskId);
          return "ended" in job
            ? end(job)
            : { status: "running", checkpoint: { phase: "firing", due, attempted: false } };
        }, context);
      },
      firing: async (task, runtime, context) => {
        const { due, attempted } = task.state.checkpoint;
        const read: { job: JobDefinition | null } = { job: null };
        await runtime.commit(async (tx) => {
          const job = await liveJob(tx, task.input.name, runtime.taskId);
          if ("ended" in job) return end(job);
          read.job = job;
          return job.action.kind === "skill" && !attempted
            ? { status: "running", checkpoint: { phase: "firing", due, attempted: true } }
            : undefined;
        }, context);
        const job = read.job;
        if (job === null) return;
        const run: JobRun = { kind: "scheduled", due };
        const outcome: JobOutcome =
          job.action.kind === "skill" && attempted ? { kind: "interrupted" } : await fireJob(job, run, deps, context);
        const firedAt = epochMs(runtime.now());
        const next = CronExpr.parse(job.cron).nextAfter(epochMs(Math.max(firedAt, due)), job.timeZone);
        await runtime.commit(async (tx) => {
          await recordRun(tx, job.name, runtime.taskId, { run, firedAt, outcome });
          return { status: "running", checkpoint: { phase: "sleeping", due: next } };
        }, context);
      },
    },
    abort: async (_task, runtime, context) => {
      await runtime.commit(() => ({ status: "terminal", outcome: { status: "aborted" } }), context);
    },
  });
}

/** The job when `task` is still its live task; otherwise why the task ends. */
async function liveJob(tx: Tx, name: JobName, task: TaskId): Promise<JobDefinition | JobTaskResult> {
  const job = await jobIn(tx, name);
  if (job === null) return { ended: "deleted" };
  return job.task === task ? job : { ended: "superseded" };
}

function end(result: JobTaskResult): NextState {
  return { status: "terminal", outcome: { status: "completed", result } };
}
