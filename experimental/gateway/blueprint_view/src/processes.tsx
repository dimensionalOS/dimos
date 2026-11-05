// dtop as a table under the topic rates: each of the run's processes (the coordinator, its workers and their
// children) with a minute of cpu, memory, threads, fds and io, from the gateway's GET /runs/stats.
import { heatStyle, Sparkline } from "dimos-react";
import { type ProcessRow, processRows, type StatsAnswer } from "./process_rows.ts";

const bytes = (value: number | null) => {
  if (value === null) {
    return "";
  }
  const units = ["B", "KB", "MB", "GB", "TB"];
  let at = 0;
  let shown = value;
  while (shown >= 1024 && at < units.length - 1) {
    shown /= 1024;
    at += 1;
  }
  return `${shown.toFixed(shown < 10 && at ? 1 : 0)} ${units[at]}`;
};

// io is in the tooltip: the sidebar has no room for a seventh column
const tooltip = (row: ProcessRow) =>
  row.io === null ? row.title : `${row.title}\nio: ${bytes(row.io)} read + written`;

export function ProcessTable(
  { stats, empty }: { stats: StatsAnswer | null; empty: string | null },
) {
  const rows = stats?.up ? processRows(stats) : [];
  return (
    <div className={`dr-rates processes${stats?.stale ? " stale" : ""}`}>
      <table>
        {rows.length > 0 && (
          <thead>
            <tr>
              <th>process</th>
              <th>cpu</th>
              <th />
              <th title="PSS where the OS reports it, else RSS">mem</th>
              <th title="threads">thr</th>
              <th title="open file descriptors">fds</th>
            </tr>
          </thead>
        )}
        <tbody>
          {rows.length === 0 && (
            <tr>
              <td className="dr-rates__empty" colSpan={6}>
                {empty ?? "waiting for dtop's stats…"}
              </td>
            </tr>
          )}
          {rows.map((row) => (
            <tr
              key={row.key}
              className={row.dead ? "dr-rates--quiet dead" : row.child ? "child" : undefined}
            >
              <td className="dr-rates__topic" title={tooltip(row)}>{row.label}</td>
              <td>
                <Sparkline
                  values={row.history}
                  width={40}
                  height={14}
                  min={0}
                  max={100}
                  strokeWidth={1}
                />
              </td>
              <td className="dr-rates__num" style={heatStyle(row.cpu, 100)}>
                {row.cpu.toFixed(0)}%
              </td>
              <td className="dr-rates__num">{bytes(row.memory)}</td>
              <td className="dr-rates__num">{row.threads ?? ""}</td>
              <td className="dr-rates__num">{row.fds ?? ""}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
