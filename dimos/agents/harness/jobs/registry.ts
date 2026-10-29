import type { Context } from "@earendil-works/chord/context";
import type { DocumentReader, TaskId, Tx } from "@earendil-works/pi-durable";
import type { Facts } from "#dimos/agents/shared/protocol/runtime/context.ts";
import type {
  JobAction,
  JobDefinition,
  JobName,
  JobRun,
  JobRunRecord,
} from "#dimos/agents/shared/protocol/jobs/state.ts";
import { defineDoc } from "@earendil-works/pi-durable";

const KEPT_RUNS = 20;

/** The jobs document's kind, which a commit publication names when the document is created or changed. */
export const JOBS_DOC_KIND = "dimos.jobs";

type Jobs = { jobs: { [name: string]: JobDefinition } };

const JobsDoc = defineDoc<Jobs>({
  kind: JOBS_DOC_KIND,
  version: 1,
  scope: "session",
  initial: () => ({ jobs: {} }),
});

export function parseJobName(text: string): JobName {
  if (!/^[a-z0-9_-]{1,64}$/.test(text)) {
    throw new Error(
      `Invalid job name ${JSON.stringify(text)}: lowercase letters, digits, "-" and "_", at most 64 characters`,
    );
  }
  return text as JobName;
}

/** A plain copy of a document draft's job, safe to hold after the commit; each object inside a draft is a draft. */
function copyOf(job: JobDefinition): JobDefinition {
  return JSON.parse(JSON.stringify(job)) as JobDefinition;
}

/** Whether two records are of one run, which its due time, tool call or request names. */
export function sameRun(a: JobRun, b: JobRun): boolean {
  switch (a.kind) {
    case "scheduled":
      return b.kind === "scheduled" && b.due === a.due;
    case "manual":
      return b.kind === "manual" && b.toolCall === a.toolCall;
    case "requested":
      return b.kind === "requested" && b.request === a.request;
  }
}

/** Creates the job or replaces its definition and task, keeping its run history. Called by the job tool inside its commit. */
export async function putJob(tx: Tx, job: Omit<JobDefinition, "runs">): Promise<void> {
  const doc = await tx.doc(JobsDoc);
  doc.jobs[job.name] = { ...job, runs: doc.jobs[job.name]?.runs ?? [] };
}

/** Removes the job and returns it, or null when there was none. */
export async function removeJob(tx: Tx, name: JobName): Promise<JobDefinition | null> {
  const doc = await tx.doc(JobsDoc);
  const job = doc.jobs[name];
  if (job === undefined) return null;
  const removed = copyOf(job);
  delete doc.jobs[name];
  return removed;
}

/** The job as this commit sees it. A JobTask reads it here to learn whether it is still the job's live task. */
export async function jobIn(tx: Tx, name: JobName): Promise<JobDefinition | null> {
  const doc = await tx.doc(JobsDoc);
  const job = doc.jobs[name];
  return job === undefined ? null : copyOf(job);
}

/** Appends a run, only when `task` is still the job's live task. */
export async function recordRun(tx: Tx, name: JobName, task: TaskId, record: JobRunRecord): Promise<void> {
  const doc = await tx.doc(JobsDoc);
  const job = doc.jobs[name];
  if (job === undefined || job.task !== task) return;
  job.runs.push(record);
  if (job.runs.length > KEPT_RUNS) job.runs.splice(0, job.runs.length - KEPT_RUNS);
}

/** Replaces the job's record of the same run; nothing when the job no longer holds one. */
export async function replaceRun(tx: Tx, name: JobName, record: JobRunRecord): Promise<void> {
  const runs = (await tx.doc(JobsDoc)).jobs[name]?.runs ?? [];
  const at = runs.findIndex((kept) => sameRun(kept.run, record.run));
  if (at !== -1) runs[at] = record;
}

export async function listJobs(reader: DocumentReader, context: Context): Promise<readonly JobDefinition[]> {
  const doc = await reader.snapshot(JobsDoc, context);
  return Object.values(doc?.jobs ?? {}).sort((a, b) => a.name.localeCompare(b.name));
}

function actionLine(action: JobAction): string {
  return action.kind === "notify"
    ? `notify ${JSON.stringify(action.text)}`
    : `skill ${action.skill}(${JSON.stringify(action.arguments)})`;
}

/** One `job:<name>` fact per job, its definition only, so a run does not change what the model is told. */
export async function jobFacts(reader: DocumentReader, context: Context): Promise<Facts> {
  const facts: { [key: `job:${string}`]: { section: string; line: string } } = {};
  for (const job of await listJobs(reader, context)) {
    facts[`job:${job.name}`] = {
      section: "Jobs",
      line: `${job.name}: ${JSON.stringify(job.cron)} ${job.timeZone}, ${actionLine(job.action)}`,
    };
  }
  return facts;
}
