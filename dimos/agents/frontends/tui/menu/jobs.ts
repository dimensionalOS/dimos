import { owner } from "#dimos/agents/sdk/failures.ts";
import { retryByRequestId } from "#dimos/agents/sdk/retry.ts";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { JobMark } from "#dimos/agents/shared/view-models/jobs/timeline.ts";
import type { MenuRow, MenuTab } from "#dimos/agents/frontends/tui/menu/base.ts";
import type { Ui } from "#dimos/agents/frontends/tui/session.ts";
import { axisText, laneText } from "#dimos/agents/frontends/tui/menu/lib/lane.ts";
import { clock } from "#dimos/agents/shared/view-models/time.ts";
import { jobLanes } from "#dimos/agents/shared/view-models/jobs/timeline.ts";
import { newActionRequest } from "#dimos/agents/shared/protocol/actions/request.ts";
import { plain } from "#dimos/agents/frontends/tui/menu/lib/rows.ts";

const HOUR_MS = 60 * 60 * 1000;

/** The lanes cover the last and the next 12 hours, one cell per 30 minutes. */
const WINDOW_MS = 12 * HOUR_MS;

const LANE_COLUMNS = 48;

const AXIS_EVERY_HOURS = 3;

const NAME_COLUMNS = 12;

const CRON_COLUMNS = 14;

const RUN_MARKS = { submitted: "✓", ok: "✓", error: "✖", interrupted: "⚠" } as const;

function markText(mark: JobMark): string {
  return mark.kind === "run"
    ? `${RUN_MARKS[mark.outcome]} ${clock(mark.at)} ${mark.outcome}`
    : `○ ${clock(mark.at)} due`;
}

function lanesAt(ui: Ui, now: EpochMs) {
  return jobLanes(ui.gateway.jobs.value, (now - WINDOW_MS) as EpochMs, (now + WINDOW_MS) as EpochMs);
}

/**
 * Tab 4 (requirement 11): every job as a lane over the last and next 12 hours, its runs by outcome and its due times
 * on a shared clock axis; r runs it now, d deletes it, Enter lists its runs.
 */
export const jobsTab: MenuTab = {
  title: "Jobs",
  rows: ({ ui, now }) => {
    const from = (now - WINDOW_MS) as EpochMs;
    const to = (now + WINDOW_MS) as EpochMs;
    const head = `${"".padEnd(NAME_COLUMNS)}  ${"".padEnd(5)}  ${"".padEnd(CRON_COLUMNS)}  `;
    const rows: MenuRow[] = [plain(`${head}${axisText(from, to, LANE_COLUMNS, AXIS_EVERY_HOURS)}`)];
    for (const lane of lanesAt(ui, now)) {
      const newest = lane.marks.filter((mark) => mark.kind === "run").at(-1);
      const name = lane.row.name.padEnd(NAME_COLUMNS).slice(0, NAME_COLUMNS);
      const conversation = `#${lane.row.conversation}`.padEnd(5);
      const cron = lane.row.cron.padEnd(CRON_COLUMNS).slice(0, CRON_COLUMNS);
      rows.push({
        text: `${name}  ${conversation}  ${cron}  ${laneText(lane.marks, from, now, to, LANE_COLUMNS)}`,
        level: newest === undefined ? "ok" : newest.level,
        keys: [
          {
            key: "r",
            label: "run now",
            run: async () => {
              await ui.actions.observe(
                `run job ${lane.row.name} now`,
                retryByRequestId(ui, { name: lane.row.name, request: newActionRequest() }, (args) =>
                  owner(ui.gateway).runJob(args),
                ),
              );
            },
          },
          {
            key: "d",
            label: "delete",
            run: async () => {
              await ui.actions.observe(
                `delete job ${lane.row.name}`,
                owner(ui.gateway).deleteJob({ name: lane.row.name }),
              );
            },
          },
        ],
        open: () => ({
          title: `${lane.row.name} runs`,
          lines: () => {
            const current = lanesAt(ui, Date.now() as EpochMs).find(
              (candidate) => candidate.row.name === lane.row.name,
            );
            return current === undefined ? ["This job is gone."] : [current.schedule, ...current.marks.map(markText)];
          },
          close: async () => {},
        }),
      });
    }
    return rows;
  },
};
