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
  useDesktopEditor,
} from "dimos-react";
import { useEffect, useRef, useState } from "react";
import { errorText, getJson } from "./api.ts";
import { LOG_POLL_MS, LOG_SEARCH_DEBOUNCE_MS } from "./constants.ts";
import { projectPath } from "./model.ts";

const MAX_RECORDS = 5000;

export function LogsModal(
  { runId, title, paused, onClose }: {
    runId: string;
    title: string;
    paused: boolean;
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
  useEffect(() => {
    if (paused) {
      return;
    }
    let timer: ReturnType<typeof setTimeout>;
    let live = true;
    const poll = async () => {
      const params = new URLSearchParams({ after: String(offset.current), level });
      if (query) {
        params.set("q", query);
      }
      try {
        const page = await getJson<{ offset: number; records: LogRecord[] }>(
          `history/${encodeURIComponent(runId)}/logs?${params}`,
        );
        if (!live) {
          return;
        }
        offset.current = page.offset;
        const fresh = page.records;
        setRecords((old) => fresh.length ? [...old, ...fresh].slice(-MAX_RECORDS) : old);
        setNote((old) => fresh.length || old === null ? null : "No log lines yet.");
      } catch (caught) {
        if (live) {
          setNote(errorText(caught));
        }
      }
      if (live) {
        timer = setTimeout(poll, LOG_POLL_MS);
      }
    };
    timer = setTimeout(poll, query ? LOG_SEARCH_DEBOUNCE_MS : 0);
    return () => {
      live = false;
      clearTimeout(timer);
    };
  }, [runId, level, query, paused]);
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
