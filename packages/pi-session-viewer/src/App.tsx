import { useMemo, useState } from "react";
import {
  Clock3,
  GitBranch,
  LockKeyhole,
  PanelLeftClose,
  PanelLeftOpen,
  Sparkles,
} from "lucide-react";

import { ancestryFor, coalesceToolExchanges } from "./session";
import type { PiSessionViewModel, SessionEntry } from "./types";
import { BranchRail } from "./components/BranchRail";
import { DetailsPanel } from "./components/DetailsPanel";
import { EntryCard } from "./components/EntryCard";

function formatTokenCount(value: number | undefined): string {
  if (value === undefined) return "—";
  return new Intl.NumberFormat(undefined, { notation: "compact" }).format(value);
}

export function App({ session }: { session: PiSessionViewModel }) {
  const [selectedHeadId, setSelectedHeadId] = useState(
    session.defaultHeadId,
  );
  const [inspected, setInspected] = useState<SessionEntry | null>(null);
  const [railOpen, setRailOpen] = useState(true);
  const ancestry = useMemo(
    () => ancestryFor(session, selectedHeadId),
    [session, selectedHeadId],
  );
  const pathIds = useMemo(
    () => new Set(ancestry.map((entry) => entry.id)),
    [ancestry],
  );
  const conversation = useMemo(
    () => coalesceToolExchanges(ancestry),
    [ancestry],
  );

  return (
    <div className="app-shell">
      <a className="skip-link" href="#transcript">
        Skip to transcript
      </a>
      <header className="topbar">
        <div className="brand">
          <span className="brand-mark" aria-hidden>
            <Sparkles />
          </span>
          <div>
            <span className="eyebrow">Pi evidence review</span>
            <h1>Session history</h1>
          </div>
        </div>
        <div className="session-badges" aria-label="Session status">
          <span className={`status-badge status-${session.summary.status}`}>
            {session.summary.status}
          </span>
          <span className="read-only-badge">
            <LockKeyhole aria-hidden />
            Read only
          </span>
        </div>
        <dl className="summary-grid">
          <div>
            <dt>Model</dt>
            <dd>{session.summary.model ?? "Mixed / unknown"}</dd>
          </div>
          <div>
            <dt>Tokens</dt>
            <dd>
              {formatTokenCount(session.summary.totalUsage.totalTokens)}
            </dd>
          </div>
          <div>
            <dt>Branches</dt>
            <dd>{session.summary.branchCount}</dd>
          </div>
          <div>
            <dt>Started</dt>
            <dd>{session.summary.startedAt}</dd>
          </div>
        </dl>
      </header>

      <div className={`workspace ${railOpen ? "" : "rail-collapsed"}`}>
        <div className="rail-shell">
          <button
            className="rail-toggle"
            type="button"
            onClick={() => setRailOpen((current) => !current)}
            aria-expanded={railOpen}
            aria-controls="branch-panel"
          >
            {railOpen ? (
              <PanelLeftClose aria-hidden />
            ) : (
              <PanelLeftOpen aria-hidden />
            )}
            <span>{railOpen ? "Hide tree" : "Show tree"}</span>
          </button>
          <div id="branch-panel" hidden={!railOpen}>
            <BranchRail
              entries={session.entries}
              pathIds={pathIds}
              selectedId={selectedHeadId}
              onSelect={setSelectedHeadId}
            />
          </div>
        </div>

        <main className="transcript" id="transcript" tabIndex={-1}>
          <div className="transcript-heading">
            <div>
              <span className="eyebrow">Selected ancestry</span>
              <h2>{conversation.length} conversation items</h2>
            </div>
            <div className="path-summary">
              <span>
                <GitBranch aria-hidden />
                {session.headIds.length} leaves
              </span>
              <span>
                <Clock3 aria-hidden />
                {ancestry.length} native events
              </span>
            </div>
          </div>
          <div className="entry-stack">
            {conversation.map((entry) => (
              <EntryCard
                entry={entry}
                key={entry.id}
                selected={inspected?.id === entry.id}
                onInspect={setInspected}
              />
            ))}
          </div>
        </main>

        <DetailsPanel entry={inspected} onClose={() => setInspected(null)} />
      </div>
    </div>
  );
}
