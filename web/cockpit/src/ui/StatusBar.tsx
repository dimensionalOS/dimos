import { useEffect, useState } from "react";
import type { SessionStatus } from "@dimos/sdk";
import type { PageTab } from "../layout/PageView.tsx";
import styles from "./StatusBar.module.css";

/** What the page shows below the header: the panel layout, or the raw channel table. */
export type View = "panels" | "channels";

const VIEWS: readonly View[] = ["panels", "channels"];

/** Wall clock ticking at `periodMs` while `active`; frozen otherwise. */
function useNowWhile(active: boolean, periodMs: number): number {
  const [now, setNow] = useState(() => Date.now());
  useEffect(() => {
    if (!active) return;
    setNow(Date.now());
    const id = setInterval(() => setNow(Date.now()), periodMs);
    return () => clearInterval(id);
  }, [active, periodMs]);
  return now;
}

const PHASE_LABEL: Record<string, string> = {
  connecting: "connecting",
  connected: "connected",
  reconnecting: "reconnecting",
  failed: "failed",
};

export function StatusBar({ status, view, onViewChange, pages, page, onPageChange }: {
  status: SessionStatus;
  view: View;
  onViewChange: (view: View) => void;
  /** The manifest's page tabs (none: no tab strip), the open page, and the pick. */
  pages: PageTab[];
  page: string | null;
  onPageChange: (id: string | null) => void;
}) {
  const transport = status.transport;
  const now = useNowWhile(transport.phase === "reconnecting", 250);

  let detail = "";
  if (transport.phase === "reconnecting") {
    const secs = Math.max(0, transport.retryAtMs - now) / 1000;
    detail = `retry in ${secs.toFixed(1)} s (attempt ${transport.attempt})`;
    if (transport.reason !== undefined) detail += ` - ${transport.reason}`;
  } else if (transport.phase === "connecting" && transport.attempt > 1) {
    detail = `attempt ${transport.attempt}`;
  }

  return (
    <header className={styles.bar}>
      <span className={styles.brand}>
        DimOS <span className={styles.brandSub}>Cockpit</span>
      </span>
      {pages.length > 0 && (
        <div role="tablist" aria-label="pages" className={styles.pages}>
          <button
            type="button"
            role="tab"
            aria-selected={view === "panels" && page === null}
            className={view === "panels" && page === null ? styles.tabActive : styles.tab}
            data-testid="tab-overview"
            onClick={() => onPageChange(null)}
          >
            Overview
          </button>
          {pages.map((tab) => {
            // The channels view covers every page: no tab reads as open then.
            const open = view === "panels" && page === tab.id;
            return (
              <button
                key={tab.id}
                type="button"
                role="tab"
                aria-selected={open}
                className={open ? styles.tabActive : styles.tab}
                data-testid={`tab-page-${tab.id}`}
                onClick={() => onPageChange(tab.id)}
              >
                {tab.label}
              </button>
            );
          })}
        </div>
      )}
      <div role="tablist" aria-label="view" className={styles.views}>
        {VIEWS.map((v) => (
          <button
            key={v}
            type="button"
            role="tab"
            aria-selected={view === v}
            className={view === v ? styles.viewActive : styles.view}
            data-testid={`view-${v}`}
            onClick={() => onViewChange(v)}
          >
            {v}
          </button>
        ))}
      </div>
      <span className={styles.pill} data-testid="status" data-phase={transport.phase}>
        {PHASE_LABEL[transport.phase]}
      </span>
      {detail !== "" && <span className={styles.detail}>{detail}</span>}
      <span className={styles.robot} data-testid="robot">
        {status.watchedRobot !== null
          ? (
            <>
              {status.watchedRobot.name}{" "}
              <span className={styles.model}>({status.watchedRobot.model})</span>
            </>
          )
          : "no robot"}
      </span>
      {status.lastError !== null && <span className={styles.error}>{status.lastError.message}
      </span>}
    </header>
  );
}
