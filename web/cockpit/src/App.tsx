import { useEffect, useState } from "react";
import type { Session } from "@dimos/sdk";
import { teleopHooks } from "@dimos/sdk/internal/teleop";
import { useStatus } from "@dimos/sdk/react";
import { LayoutTree } from "./layout/LayoutTree.tsx";
import { type PageTab, pageTabs, PageView } from "./layout/PageView.tsx";
import { startChatTranscripts } from "./panels/chatTranscript.ts";
import { ChannelList } from "./ui/ChannelList.tsx";
import { RobotPicker } from "./ui/RobotPicker.tsx";
import { StatusBar, type View } from "./ui/StatusBar.tsx";
import { TokenForm } from "./ui/TokenForm.tsx";
import { clearToken, readToken, storeToken } from "./token.ts";
import styles from "./App.module.css";

export function App({ session }: { session: Session }) {
  const status = useStatus(session);
  const teleop = teleopHooks(session);
  // Panels or the raw channel table, the open page tab, and a pending
  // "switch robot" request. All live above the epoch-keyed <main> so a
  // manifest change (robot restart or switch) does not yank the operator off
  // the table, their page, or the picker.
  const [view, setView] = useState<View>("panels");
  const [pageId, setPageId] = useState<string | null>(null);
  const [picking, setPicking] = useState(false);
  const watchedId = status.watchedRobot?.id ?? null;
  const hasMultipleRobots = status.robots.length > 1;
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

  // Nothing to switch to (the list shrank, or a relay restart pushed an empty
  // one): drop a stale request so a later registration does not pop the
  // picker over the live layout unprompted.
  useEffect(() => {
    if (!hasMultipleRobots) setPicking(false);
  }, [hasMultipleRobots]);

  const openPage = (id: string | null): void => {
    setPageId(id);
    setView("panels");
  };

  const pickRobot = (id: string): void => {
    setPicking(false);
    // The promise rejects only with WatchRejectedError (superseded by a newer
    // pick, or the session closed): nothing the operator can act on.
    void session.watch(id).catch(() => {});
  };

  // Several robots and none watched: the operator has to pick (the SDK
  // auto-watches only a lone robot). A pick pins the watch for the session;
  // "switch robot" in the status bar reopens the list.
  const showPicker = hasMultipleRobots && (picking || status.watchedRobot === null);

  // Token changes reload the page instead of re-creating the session: that
  // keeps the single connect() in main.tsx.
  const logOut = () => {
    clearToken();
    location.reload();
  };

  let content;
  let pages: PageTab[] = [];
  let page: string | null = null;
  if (status.transport.phase === "failed") {
    content = status.transport.code === "auth_failed"
      ? (
        <TokenForm
          message={status.transport.reason}
          onSubmit={(token) => {
            storeToken(token);
            location.reload();
          }}
        />
      )
      : <p className={styles.notice}>Connection failed: {status.transport.reason}</p>;
  } else if (showPicker) {
    content = <RobotPicker robots={status.robots} current={watchedId} onPick={pickRobot} />;
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
        onSwitchRobot={hasMultipleRobots && !showPicker ? () => setPicking(true) : null}
        onLogOut={readToken() !== null ? logOut : null}
      />
      {/* A changed manifest remounts everything below the status bar. */}
      <main className={styles.main} key={status.epoch}>
        {content}
      </main>
    </div>
  );
}
