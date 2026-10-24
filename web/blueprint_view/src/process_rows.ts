// The Processes table's rows from the gateway's GET /runs/stats (dtop's /resource_stats): pure, for the tests.

export interface ProcessSample {
  pid?: number;
  worker_id?: number;
  alive?: boolean;
  dedicated?: boolean;
  modules?: string[];
  cpu_percent?: number;
  rss?: number;
  pss?: number;
  num_threads?: number;
  num_fds?: number;
  io_read_bytes?: number;
  io_write_bytes?: number;
  children?: { pid: number; name: string; cpu_percent: number }[];
}

export interface StatsAnswer {
  up: boolean;
  age: number | null;
  stale: boolean;
  coordinator: ProcessSample | null;
  workers: ProcessSample[];
  cpuHistory: Record<string, number[]>;
}

export interface ProcessRow {
  key: string;
  label: string;
  title: string;
  child: boolean;
  dead: boolean;
  cpu: number;
  history: number[];
  /** bytes: PSS where the OS reports it (Linux), else RSS */
  memory: number | null;
  threads: number | null;
  fds: number | null;
  io: number | null;
}

/** The coordinator, then each worker followed by its children. */
export function processRows(stats: StatsAnswer): ProcessRow[] {
  const history = (key: string) => stats.cpuHistory[key] ?? [];
  const row = (key: string, label: string, title: string, sample: ProcessSample): ProcessRow => ({
    key,
    label,
    title,
    child: false,
    dead: sample.alive === false,
    cpu: sample.cpu_percent ?? 0,
    history: history(key),
    memory: sample.pss || sample.rss || null,
    threads: sample.num_threads ?? null,
    fds: sample.num_fds ?? null,
    io: sample.io_read_bytes === undefined
      ? null
      : (sample.io_read_bytes ?? 0) + (sample.io_write_bytes ?? 0),
  });
  const rows: ProcessRow[] = [];
  if (stats.coordinator) {
    rows.push(row("coordinator", "coordinator", `pid ${stats.coordinator.pid}`, stats.coordinator));
  }
  for (const worker of stats.workers) {
    const modules = (worker.modules ?? []).join(", ");
    const tags = [worker.dedicated ? "dedicated" : "", worker.alive === false ? "dead" : ""]
      .filter(Boolean).join(", ");
    rows.push(
      row(
        `worker:${worker.worker_id}`,
        `worker ${worker.worker_id}${modules ? ` · ${modules}` : ""}`,
        `pid ${worker.pid}${tags ? ` (${tags})` : ""}\n${modules || "no modules"}`,
        worker,
      ),
    );
    for (const child of worker.children ?? []) {
      rows.push({
        key: `child:${child.pid}`,
        label: `↳ ${child.name}`,
        title: `pid ${child.pid}, a child of worker ${worker.worker_id}`,
        child: true,
        dead: false,
        cpu: child.cpu_percent,
        history: history(`child:${child.pid}`),
        memory: null,
        threads: null,
        fds: null,
        io: null,
      });
    }
  }
  return rows;
}
