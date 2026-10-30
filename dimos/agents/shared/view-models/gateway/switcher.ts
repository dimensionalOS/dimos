import type { ConversationId } from "@earendil-works/pi-durable";
import type { ConversationRow, Running } from "../../protocol/conversation/summary.ts";
import type { EpochMs } from "../../protocol/time.ts";
import type { Level } from "../levels.ts";
import { clean } from "../text.ts";

/** One switcher row as a line: `› #7  running bash 41s  walk to the kitchen door…  ⚡1`, forks indented. */
export function switcherLine(row: SwitcherRow): string {
  const tail = row.unread > 0 ? `  ⚡${row.unread}` : "";
  return `${row.current ? "›" : " "} ${"  ".repeat(row.depth)}#${row.id}  ${row.state}  ${clean(row.title)}${tail}`;
}

/** One line of the conversation switcher, in display order. */
export type SwitcherRow = {
  readonly id: ConversationId;
  /** 0 for a root; a fork is one deeper than its source. */
  readonly depth: number;
  readonly title: string;
  /** "running bash 12s", "answering", "compaction", "summary", "queue 2", "idle". */
  readonly state: string;
  readonly unread: number;
  readonly current: boolean;
  readonly level: Level;
};

/** "12s" under a minute, "2m" under an hour, "3h" after. */
function ageText(ms: number): string {
  const seconds = Math.max(0, Math.floor(ms / 1000));
  if (seconds < 60) return `${seconds}s`;
  const minutes = Math.floor(seconds / 60);
  if (minutes < 60) return `${minutes}m`;
  return `${Math.floor(minutes / 60)}h`;
}

function stateText(running: Running | null, queued: number, now: EpochMs): string {
  if (running === null) return queued > 0 ? `queue ${queued}` : "idle";
  switch (running.kind) {
    case "tool":
      return `running ${running.name} ${ageText(now - running.since)}`;
    case "answering":
    case "compaction":
    case "summary":
      return running.kind;
  }
}

/** Roots newest first, each followed by its forks (by `parent`), recursively. A row whose source is gone is a root. */
export function switcherRows(
  conversations: readonly ConversationRow[],
  current: ConversationId | null,
  now: EpochMs,
): readonly SwitcherRow[] {
  const known = new Set(conversations.map((row) => row.id));
  const children = new Map<ConversationId | null, ConversationRow[]>();
  for (const row of conversations) {
    const source = row.parent !== null && known.has(row.parent.conversation) ? row.parent.conversation : null;
    const siblings = children.get(source);
    if (siblings === undefined) children.set(source, [row]);
    else siblings.push(row);
  }
  const rows: SwitcherRow[] = [];
  const walk = (source: ConversationId | null, depth: number): void => {
    const siblings = children.get(source);
    if (siblings === undefined) return;
    for (const row of [...siblings].sort((a, b) => b.updatedAt - a.updatedAt)) {
      const idleWithQueue = row.running === null && row.queued > 0;
      rows.push({
        id: row.id,
        depth,
        title: row.title,
        state: stateText(row.running, row.queued, now),
        unread: row.unread,
        current: row.id === current,
        level: row.unread > 0 || idleWithQueue ? "warn" : "ok",
      });
      walk(row.id, depth + 1);
    }
  };
  walk(null, 0);
  return rows;
}
