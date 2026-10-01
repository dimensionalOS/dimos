// Stats page: dtop in a tab. The resource monitor's snapshot arrives as one
// stats.json.v1 frame per tick (1 Hz) on a latest channel; the table shows
// the coordinator, then every worker with its children indented beneath,
// each with a rolling CPU sparkline. The history is the page's own: Tabs
// unmounts inactive pages, so the window restarts when the tab is reopened.
// When frames stop the whole table dims (dtop does the same) and the badge
// says for how long.

import { Fragment, type ReactNode, useEffect, useRef, useState } from "react";
import type { PanelSpec } from "@dimos/shared";
import type { ChannelStore } from "@dimos/sdk";
import { useStoreChannel } from "@dimos/sdk/react";
import { Badge, PanelFrame } from "../layout/PanelFrame.tsx";
import type { PanelProps } from "./registry.tsx";
import { drawSparkline } from "./sparkline.ts";
import {
  childKey,
  COORDINATOR_KEY,
  cpuColor,
  EMPTY_SNAPSHOT,
  fmtIo,
  fmtMem,
  fmtPct,
  fmtSecs,
  foldSnapshot,
  type ProcessStats,
  type StatsSnapshot,
  WINDOW,
  workerKey,
} from "./stats.ts";
import styles from "./StatsPanel.module.css";

/** Two missed 1 Hz snapshots: the monitor, or its process, is gone. */
export const STATS_STALE_MS = 3000;

const HEADERS = [
  "process",
  "modules",
  `cpu (${WINDOW} s)`,
  "cpu",
  "pss",
  "threads",
  "fds",
  "children",
  "user",
  "sys",
  "iowait",
  "io r/w",
];
const TEXT_COLUMNS = 3;

function Sparkline({ samples }: { samples: readonly number[] }) {
  const ref = useRef<HTMLCanvasElement | null>(null);
  useEffect(() => {
    const canvas = ref.current;
    const ctx = canvas === null ? null : canvas.getContext("2d");
    if (canvas === null || ctx === null) return;
    const dpr = globalThis.devicePixelRatio || 1;
    // clientWidth is 0 before layout (and under happy-dom): keep the
    // attribute size then.
    const w = Math.round(canvas.clientWidth * dpr) || canvas.width;
    const h = Math.round(canvas.clientHeight * dpr) || canvas.height;
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    const newest = samples[samples.length - 1] ?? 0;
    drawSparkline(ctx, samples, WINDOW, w, h, cpuColor(newest), dpr);
  }, [samples]);
  return <canvas ref={ref} className={styles.spark} width={96} height={20} aria-hidden="true" />;
}

function Metrics({ stats }: { stats: ProcessStats }) {
  const cells = [
    fmtMem(stats.pss),
    String(stats.num_threads),
    String(stats.num_fds),
    String(stats.num_children),
    fmtSecs(stats.cpu_time_user),
    fmtSecs(stats.cpu_time_system),
    fmtSecs(stats.cpu_time_iowait),
    fmtIo(stats.io_read_bytes, stats.io_write_bytes),
  ];
  return (
    <>
      {cells.map((text, i) => (
        <td key={i} className={styles.num}>
          {text}
        </td>
      ))}
    </>
  );
}

/** One table row: a process (coordinator or live worker, all metrics), a
 * dead worker (dashes: the producer sends zeros and pid 0), or a child
 * process (CPU only, indented). */
function Row({ testId, name, pid, tags, modules, kind, cpu, samples, stats }: {
  testId: string;
  name: string;
  pid: number;
  tags?: ReactNode;
  modules?: string;
  kind: "process" | "dead" | "child";
  cpu: number;
  samples: readonly number[];
  stats?: ProcessStats;
}) {
  const dead = kind === "dead";
  const filler = dead ? "-" : "";
  return (
    <tr
      className={dead ? styles.dead : kind === "child" ? styles.child : undefined}
      data-testid={testId}
      data-dead={dead || undefined}
    >
      <td className={styles.name}>
        {kind === "child" && "↳ "}
        {name}
        {pid > 0 && <span className={styles.pid}>[{pid}]</span>}
        {tags}
      </td>
      <td className={styles.modules} title={modules}>
        {modules}
      </td>
      <td>{!dead && <Sparkline samples={samples} />}</td>
      <td className={styles.cpu} style={dead ? undefined : { color: cpuColor(cpu) }}>
        {dead ? filler : fmtPct(cpu)}
      </td>
      {stats !== undefined && !dead ? <Metrics stats={stats} /> : (
        Array.from(
          { length: HEADERS.length - TEXT_COLUMNS - 1 },
          (_, i) => (
            <td key={i} className={styles.num}>
              {filler}
            </td>
          ),
        )
      )}
    </tr>
  );
}

function StatsTable({ ch, snapshot }: { ch: string; snapshot: StatsSnapshot }) {
  const frame = snapshot.frame;
  if (frame === null) return null;
  const samples = (key: string): readonly number[] => snapshot.cpu.get(key) ?? [];
  return (
    <table className={styles.table} data-testid={`stats-${ch}-table`}>
      <thead>
        <tr>
          {HEADERS.map((label, i) => (
            <th key={label} className={i < TEXT_COLUMNS ? undefined : styles.num}>
              {label}
            </th>
          ))}
        </tr>
      </thead>
      <tbody>
        <Row
          testId={`stats-${ch}-row-coordinator`}
          name="coordinator"
          pid={frame.coordinator.pid}
          kind="process"
          cpu={frame.coordinator.cpu_percent}
          samples={samples(COORDINATOR_KEY)}
          stats={frame.coordinator}
        />
        {frame.workers.map((worker) => (
          <Fragment key={worker.worker_id}>
            <Row
              testId={`stats-${ch}-row-worker-${worker.worker_id}`}
              name={`worker ${worker.worker_id}`}
              pid={worker.pid}
              tags={
                <>
                  {worker.dedicated && <span className={styles.chip}>dedicated</span>}
                  {!worker.alive && <span className={styles.chipDead}>dead</span>}
                </>
              }
              modules={worker.modules.join(", ")}
              kind={worker.alive ? "process" : "dead"}
              cpu={worker.cpu_percent}
              samples={samples(workerKey(worker.worker_id))}
              stats={worker}
            />
            {worker.children.map((child) => (
              <Row
                key={child.pid}
                testId={`stats-${ch}-row-child-${child.pid}`}
                name={child.name}
                pid={child.pid}
                kind="child"
                cpu={child.cpu_percent}
                samples={samples(childKey(child.pid))}
              />
            ))}
          </Fragment>
        ))}
      </tbody>
    </table>
  );
}

/** The channel's frames folded into per-row CPU history while mounted, one
 * sample per frame (direct store notifications, not the UI tick); a store
 * reset (producer gone or changed) empties it. */
function useStatsHistory(store: ChannelStore, ch: string): StatsSnapshot {
  const [snapshot, setSnapshot] = useState(EMPTY_SNAPSHOT);
  useEffect(() => {
    let seen = 0;
    let current = EMPTY_SNAPSHOT;
    const pull = (): void => {
      const slot = store.get(ch);
      if (slot === null) {
        if (seen === 0) return;
        seen = 0;
        current = EMPTY_SNAPSHOT;
      } else if (slot.version > seen) {
        seen = slot.version;
        current = foldSnapshot(current, slot.value);
      } else {
        return;
      }
      setSnapshot(current);
    };
    const unsubscribe = store.subscribe(ch, pull);
    pull(); // a slot may predate the mount
    return unsubscribe;
  }, [store, ch]);
  return snapshot;
}

function StatsPage({ spec, store }: { spec: PanelSpec; store: ChannelStore }) {
  const ch = spec.channels[0];
  const snapshot = useStatsHistory(store, ch);
  const { stats } = useStoreChannel(store, ch);
  const stale = stats.ageMs !== null && stats.ageMs > STATS_STALE_MS;
  const badge = (
    <Badge
      store={store}
      ch={ch}
      staleMs={STATS_STALE_MS}
      unit="Hz"
      testId={`stats-${ch}-badge`}
    />
  );
  return (
    <PanelFrame spec={spec} badge={badge}>
      <div className={styles.page} data-testid={`stats-${ch}`} data-stale={stale || undefined}>
        {snapshot.unreadable && (
          <div className={styles.error} role="alert">
            unreadable stats frame
          </div>
        )}
        {snapshot.frame === null
          ? <span className={styles.hint}>waiting for resource stats...</span>
          : <StatsTable ch={ch} snapshot={snapshot} />}
      </div>
    </PanelFrame>
  );
}

export function StatsPanel({ spec, store }: PanelProps) {
  if (spec.channels.length !== 1) {
    return (
      <PanelFrame spec={spec}>
        <span className={styles.hint}>stats panel {spec.id}: no stats channel bound</span>
      </PanelFrame>
    );
  }
  return <StatsPage spec={spec} store={store} />;
}
