// A run's structured log (GET runs/log_hist/{id}), followed live unless paused; the server filters by level and search.
// A launch that never became a run has no such log: its own output stands in (LaunchOutputModal).
import {
  CopyButton,
  EditorNote,
  isTraceback,
  type LogRecord,
  Modal,
  StructuredLog,
  Text,
  Traceback,
  useConnectionState,
  useDesktopEditor,
  useZenoh,
} from "dimos-react";
import { useEffect, useRef, useState } from "react";
import { errorText, getJson } from "./api.ts";
import { oneInFlight, useEvery, type ViewConfig } from "./live.ts";
import { projectPath } from "./model.ts";

export function LogsModal(
  { runId, title, paused, config, onClose }: {
    runId: string;
    title: string;
    paused: boolean;
    config: ViewConfig | null;
    onClose: () => void;
  },
) {
  const [level, setLevel] = useState("info");
  const [query, setQuery] = useState("");
  const [follow, setFollow] = useState(true);
  const [records, setRecords] = useState<LogRecord[]>([]);
  const [note, setNote] = useState<string | null>("loading…");
  const editor = useDesktopEditor();
  const offset = useRef(0);
  useEffect(() => {
    offset.current = 0;
    setRecords([]);
    setNote("loading…");
  }, [runId, level, query]);
  const zenoh = useZenoh();
  const eventsArrive = useConnectionState() === "connected";
  const maxRecords = useRef(Infinity);
  maxRecords.current = config?.logMaxRecords ?? Infinity;
  const [read, setRead] = useState<(() => void) | null>(null);
  // a run logging where the gateway doesn't watch (a third checkout's logs/) sends no log events: polled instead
  const [watched, setWatched] = useState(true);
  // the next lines are asked for on each log event for this run; a new level or search cancels the request still
  // out for the old one, so the newest always wins
  useEffect(() => {
    if (paused) {
      return;
    }
    const abort = new AbortController();
    const readNext = oneInFlight(async () => {
      const params = new URLSearchParams({ after: String(offset.current), level });
      if (query) {
        params.set("q", query);
      }
      try {
        const page = await getJson<{ offset: number; records: LogRecord[]; watched?: boolean }>(
          `history/${encodeURIComponent(runId)}/logs?${params}`,
          { signal: abort.signal },
        );
        if (abort.signal.aborted) {
          return;
        }
        offset.current = page.offset;
        setWatched(page.watched ?? true);
        const fresh = page.records;
        setRecords((old) => fresh.length ? [...old, ...fresh].slice(-maxRecords.current) : old);
        setNote((old) => fresh.length || old === null ? null : "No log lines yet.");
      } catch (caught) {
        if (!abort.signal.aborted) {
          setNote(errorText(caught));
        }
      }
    });
    readNext();
    setRead(() => readNext);
    const offLog = zenoh?.subscribeDimos("log", (event) => event?.run === runId && readNext());
    const offReconnect = zenoh?.onReconnect(readNext);
    return () => {
      abort.abort();
      offLog?.();
      offReconnect?.();
      setRead(null);
    };
  }, [runId, level, query, paused, zenoh]);
  useEvery(read ?? (() => {}), read && (!eventsArrive || !watched) ? config?.pollWithoutEventsS : null);
  return (
    <Modal
      open
      onClose={onClose}
      size="xl"
      title={
        <>
          Logs · {title} <span className="sub">{runId}</span>
        </>
      }
      data-bp-logs-modal
    >
      <StructuredLog
        className="fill"
        records={records}
        note={note}
        level={level}
        onLevelChange={setLevel}
        query={query}
        onQueryChange={setQuery}
        follow={follow}
        onFollowChange={setFollow}
        banner={<EditorNote status={editor.status} />}
        onOpenFile={editor.open ?? undefined}
        formatPath={projectPath}
      />
    </Modal>
  );
}

/**
 * What a launch printed, for one that never became a run (a bad config, a build that failed): dimos writes no run
 * log for it, so this output is the only account of it there is. A traceback is drawn frame by frame, anything else
 * as it came.
 */
export function LaunchOutputModal(
  { output, why, title, onClose }: {
    output: string;
    why: string;
    title: string;
    onClose: () => void;
  },
) {
  const editor = useDesktopEditor();
  const text = output.trimEnd();
  return (
    <Modal
      open
      onClose={onClose}
      size="xl"
      title={
        <>
          Launch output · {title} <span className="sub">no run started</span>
        </>
      }
      footer={<CopyButton text={text || why} label="Copy output" />}
      data-bp-launch-output
    >
      <div className="launch-output">
        <Text tone="warn" className="launch-why">{why}</Text>
        {isTraceback(text)
          ? (
            <Traceback
              className="fill"
              text={text}
              onOpenFile={editor.open ?? undefined}
              formatPath={projectPath}
            />
          )
          : <pre className="launch-raw">{text || "it printed nothing."}</pre>}
        <EditorNote status={editor.status} />
      </div>
    </Modal>
  );
}
