import { X } from "lucide-react";

import type { SessionEntry } from "../types";

export function DetailsPanel({
  entry,
  onClose,
}: {
  entry: SessionEntry | null;
  onClose: () => void;
}) {
  return (
    <aside
      className={`details-panel ${entry ? "details-open" : ""}`}
      aria-label="Entry details"
      aria-hidden={!entry}
    >
      {entry && (
        <>
          <header className="details-header">
            <div>
              <span className="eyebrow">Forensic detail</span>
              <h2>{entry.title}</h2>
            </div>
            <button
              className="icon-button"
              type="button"
              onClick={onClose}
              aria-label="Close entry details"
            >
              <X aria-hidden />
            </button>
          </header>
          <dl className="detail-list">
            <div>
              <dt>Entry ID</dt>
              <dd>{entry.id}</dd>
            </div>
            <div>
              <dt>Parent ID</dt>
              <dd>{entry.parentId ?? "Root"}</dd>
            </div>
            <div>
              <dt>Type</dt>
              <dd>{entry.type}</dd>
            </div>
            <div>
              <dt>Timestamp</dt>
              <dd>{entry.timestamp}</dd>
            </div>
            {entry.provider && (
              <div>
                <dt>Provider</dt>
                <dd>{entry.provider}</dd>
              </div>
            )}
            {entry.usage && (
              <div>
                <dt>Usage</dt>
                <dd>
                  <pre>{JSON.stringify(entry.usage, null, 2)}</pre>
                </dd>
              </div>
            )}
          </dl>
          <details className="raw-entry">
            <summary>Raw native entry</summary>
            <pre>
              <code>{JSON.stringify(entry.raw, null, 2)}</code>
            </pre>
          </details>
        </>
      )}
    </aside>
  );
}
