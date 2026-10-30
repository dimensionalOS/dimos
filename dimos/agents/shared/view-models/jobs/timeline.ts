import type { EpochMs } from "../../protocol/time.ts";
import type { JobRow } from "../../protocol/jobs/state.ts";
import type { Level } from "../levels.ts";

/** One mark on a job's lane: a run that happened, with its outcome's level, or a due time to come. */
export type JobMark =
  | {
      readonly kind: "run";
      readonly at: EpochMs;
      readonly outcome: "submitted" | "ok" | "error" | "interrupted";
      readonly level: Level;
    }
  | { readonly kind: "due"; readonly at: EpochMs };

/** Requirement 11 as data: per job its schedule line and its marks between `from` and `to`, oldest first. */
export type JobLane = {
  readonly row: JobRow;
  /** `"0 * * * *" Europe/Berlin · notify "Walk the patrol route"` */
  readonly schedule: string;
  readonly marks: readonly JobMark[];
};

const OUTCOME_LEVELS: { readonly [K in JobRow["runs"][number]["outcome"]["kind"]]: Level } = {
  submitted: "ok",
  ok: "ok",
  error: "critical",
  interrupted: "warn",
};

function scheduleOf(row: JobRow): string {
  const { action } = row;
  const what =
    action.kind === "notify"
      ? `notify ${JSON.stringify(action.text)}`
      : `skill ${action.skill}(${JSON.stringify(action.arguments)})`;
  return `${JSON.stringify(row.cron)} ${row.timeZone} · ${what}`;
}

export function jobLanes(jobs: readonly JobRow[], from: EpochMs, to: EpochMs): readonly JobLane[] {
  const within = (at: EpochMs): boolean => at >= from && at <= to;
  return jobs.map((row) => {
    const marks: JobMark[] = [];
    for (const run of row.runs) {
      if (!within(run.firedAt)) continue;
      marks.push({ kind: "run", at: run.firedAt, outcome: run.outcome.kind, level: OUTCOME_LEVELS[run.outcome.kind] });
    }
    const due = row.nextDue === null ? row.upcoming : [row.nextDue, ...row.upcoming];
    for (const at of due) if (within(at)) marks.push({ kind: "due", at });
    marks.sort((a, b) => a.at - b.at);
    return { row, schedule: scheduleOf(row), marks };
  });
}
