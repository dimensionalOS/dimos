import type { Harness, TaskId, TaskRecord } from "@earendil-works/pi-durable";
import type { Cell, Ctx, Self } from "@earendil-works/chord";
import type { Principal } from "#dimos/agents/gateway/auth/principal.ts";
import { BACKGROUND_CONTEXT, withAbortSignal } from "@earendil-works/chord/context";
import { GatewayState } from "#dimos/agents/shared/protocol/gateway/contract.ts";
import { commitEvents } from "#dimos/agents/gateway/server/objects/lib/commits.ts";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { JobDefinition, JobRow } from "#dimos/agents/shared/protocol/jobs/state.ts";
import type { JsonValue } from "@earendil-works/chord";
import { CronExpr } from "#dimos/agents/harness/jobs/cron.ts";
import { JOBS_DOC_KIND, listJobs } from "#dimos/agents/harness/jobs/registry.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";

const UPCOMING = 3;

/** The due time a job task's checkpoint names; null once the task ended. */
function dueOf(record: TaskRecord<JsonValue, JsonValue, unknown> | undefined): EpochMs | null {
  if (record === undefined) return null;
  const { state } = record;
  if (state.status !== "pending" && state.status !== "running" && state.status !== "waiting") return null;
  const { checkpoint } = state;
  if (typeof checkpoint !== "object" || checkpoint === null || Array.isArray(checkpoint)) return null;
  const { due } = checkpoint;
  return typeof due === "number" ? epochMs(due) : null;
}

function upcomingOf(job: JobDefinition, nextDue: EpochMs | null): readonly EpochMs[] {
  if (nextDue === null) return [];
  const cron = CronExpr.parse(job.cron);
  const times: EpochMs[] = [];
  let after = nextDue;
  for (let index = 0; index < UPCOMING; index += 1) {
    after = cron.nextAfter(after, job.timeZone);
    times.push(after);
  }
  return times;
}

/** The row returned by job actions and published in the gateway's jobs cell. */
export function jobRow(job: JobDefinition, nextDue: EpochMs | null): JobRow {
  return {
    name: job.name,
    cron: job.cron,
    timeZone: job.timeZone,
    action: job.action,
    conversation: job.conversation,
    createdAt: job.createdAt,
    runs: job.runs,
    nextDue,
    upcoming: upcomingOf(job, nextDue),
  };
}

type JobIndex = {
  readonly events: ReturnType<typeof commitEvents>;
  jobs: readonly JobDefinition[];
  readonly due: Map<TaskId, EpochMs | null>;
  refreshing: Promise<void> | null;
  dirty: boolean;
};

/** Job definitions and task checkpoints projected into the gateway's observable rows. */
export function jobRows(self: Self<Principal>, harness: Harness): Cell<readonly JobRow[]> {
  const index = self.resource("jobIndex", {
    version: 1,
    open: (): JobIndex => ({ events: commitEvents(harness), jobs: [], due: new Map(), refreshing: null, dirty: false }),
    close: (index) => index.events.close(),
  });
  const currentRows = () => index.value.jobs.map((job) => jobRow(job, index.value.due.get(job.task) ?? null));
  const load = async (ctx: Ctx<Principal>): Promise<void> => {
    const context = withAbortSignal(ctx.signal, BACKGROUND_CONTEXT);
    const state = index.value;
    state.jobs = await listJobs(harness, context);
    for (const job of state.jobs) {
      if (state.due.has(job.task)) continue;
      const task = await harness.getTask(job.task, context);
      // A commit delivered during the read has the newer checkpoint.
      if (!state.due.has(job.task)) state.due.set(job.task, dueOf(task));
    }
    const tasks = new Set(state.jobs.map((job) => job.task));
    for (const task of state.due.keys()) if (!tasks.has(task)) state.due.delete(task);
  };
  const rows = self.state("jobs", GatewayState.jobs, {
    init: async (ctx) => {
      await load(ctx);
      return currentRows();
    },
  });
  const refresh = (ctx: Ctx<Principal>): Promise<void> => {
    const state = index.value;
    state.dirty = true;
    if (state.refreshing !== null) return state.refreshing;
    // Document invalidations arrive on the Session line. Read the snapshot after that commit returns.
    state.refreshing = Promise.resolve()
      .then(async () => {
        do {
          state.dirty = false;
          await load(ctx);
          if (ctx.signal.aborted) return;
          rows.replace(ctx, currentRows());
        } while (state.dirty);
      })
      .finally(() => {
        state.refreshing = null;
      });
    return state.refreshing;
  };
  self.on(
    "jobCommits",
    index,
    (index) => index.events,
    (publication, ctx) => {
      const state = index.value;
      let changed = false;
      let definitionsChanged = false;
      for (const change of publication.changes) {
        if (change.type === "task" && state.jobs.some((job) => job.task === change.value.id)) {
          state.due.set(change.value.id, dueOf(change.value));
          changed = true;
        } else if (change.type === "document" && change.record.kind === JOBS_DOC_KIND) definitionsChanged = true;
      }
      if (changed) rows.replace(ctx, currentRows());
      if (definitionsChanged) return refresh(ctx);
    },
  );
  return rows;
}
