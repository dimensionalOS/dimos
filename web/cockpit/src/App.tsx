import { useEffect, useState } from "react";
import type { Session } from "@dimos/sdk";
import { teleopHooks } from "@dimos/sdk/internal/teleop";
import { useStatus } from "@dimos/sdk/react";
import { LayoutTree } from "./layout/LayoutTree.tsx";
import { type PageTab, pageTabs, PageView } from "./layout/PageView.tsx";
import { startChatTranscripts } from "./panels/chatTranscript.ts";
import { ChannelList } from "./ui/ChannelList.tsx";
import { StatusBar, type View } from "./ui/StatusBar.tsx";
import styles from "./App.module.css";

export function App({ session }: { session: Session }) {
  const status = useStatus(session);
  const teleop = teleopHooks(session);
  // Panels or the raw channel table, and the open page tab. Both live above
  // the epoch-keyed <main> so a manifest change (robot restart) does not yank
  // the operator off the table or their page.
  const [view, setView] = useState<View>("panels");
  const [pageId, setPageId] = useState<string | null>(null);
  // Chat transcripts outlive their panels (an inactive page is unmounted),
  // so they start as soon as a manifest names them.
  useEffect(() => {
    if (status.manifest !== null) startChatTranscripts(session.store, status.manifest);
  }, [session, status.manifest]);

  useEffect(() => {
    if (
      status.manifest !== null && pageId !== null && !status.manifest.pages.includes(pageId)
    ) {
      setPageId(null);
    }
  }, [pageId, status.manifest]);

  const openPage = (id: string | null): void => {
    setPageId(id);
    setView("panels");
  };

  let content;
  let pages: PageTab[] = [];
  let page: string | null = null;
  if (status.transport.phase === "failed") {
    content = <p className={styles.notice}>Connection failed: {status.transport.reason}</p>;
  } else if (status.robots.length > 1) {
    content = (
      <p className={styles.notice}>
        {status.robots.length} robots connected; the robot picker arrives in a later release.
      </p>
    );
  } else if (status.manifestUnsupported) {
    content = (
      <p className={styles.notice}>
        This robot's software is newer than this Cockpit build. Reload the page to pick up the
        latest Cockpit.
      </p>
    );
  } else if (status.manifest === null || status.manifest.channels.length === 0) {
    content = <p className={styles.notice}>Waiting for a robot to register...</p>;
  } else {
    pages = pageTabs(status.manifest);
    // A page the new manifest no longer has falls back to the grid.
    if (pageId !== null && pages.some((p) => p.id === pageId)) page = pageId;
    if (view === "channels") {
      content = (
        <ChannelList
          channels={status.manifest.channels}
          panels={status.manifest.panels}
          store={session.store}
        />
      );
    } else if (status.manifest.panels.length === 0) {
      // cockpit(channels=[...]) alone: nothing to lay out, only rows.
      content = <p className={styles.notice}>This cockpit has no panels. Open the channels tab.</p>;
    } else {
      content = (
        <PageView
          manifest={status.manifest}
          page={page}
          store={session.store}
          teleop={teleop}
          session={session}
        >
          <LayoutTree
            manifest={status.manifest}
            store={session.store}
            teleop={teleop}
            session={session}
          />
        </PageView>
      );
    }
  }

  return (
    <div className={styles.app}>
      <StatusBar
        status={status}
        view={view}
        onViewChange={setView}
        pages={pages}
        page={page}
        onPageChange={openPage}
      />
      {/* A changed manifest remounts everything below the status bar. */}
      <main className={styles.main} key={status.epoch}>
        {content}
      </main>
    </div>
  );
}
