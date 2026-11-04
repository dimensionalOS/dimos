import type { MenuRow, MenuTab } from "#dimos/agents/frontends/tui/menu/base.ts";
import { actionLine } from "#dimos/agents/shared/view-models/actions.ts";
import { connectionSegment } from "#dimos/agents/shared/view-models/connection.ts";
import { linkSegments, runSegment } from "#dimos/agents/shared/view-models/gateway/link.ts";
import { plain } from "#dimos/agents/frontends/tui/menu/lib/rows.ts";
import { worst } from "#dimos/agents/shared/view-models/levels.ts";

/** The newest refused or unknown actions the overview keeps in view. */
const REFUSED_SHOWN = 5;

/** Tab 1: this terminal's gateway link, the DimOS link, load and run, the newest refused actions. */
export const overviewTab: MenuTab = {
  title: "Overview",
  rows: ({ ui, now }) => {
    const gateway = ui.gateway;
    const runs = gateway.runs.value;
    const link = linkSegments(gateway.link.value, gateway.load.value, runs, now);
    const run = runSegment(runs);
    const connection = connectionSegment(ui.connection.value);
    const rows: MenuRow[] = [
      plain(connection.text, connection.level),
      plain(link.map((segment) => segment.text).join(" · "), worst(link.map((segment) => segment.level))),
      run === null ? plain("No DimOS run file") : plain(run.text, run.level),
    ];
    const refused = ui.actions.value.filter((record) => record.status === "refused" || record.status === "unknown");
    for (const record of refused.slice(-REFUSED_SHOWN).reverse()) rows.push(plain(actionLine(record), "warn"));
    return rows;
  },
};
