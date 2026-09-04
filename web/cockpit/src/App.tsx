import type { Session } from "@dimos/sdk";
import { teleopHooks } from "@dimos/sdk/internal/teleop";
import { useStatus } from "@dimos/sdk/react";
import { LayoutTree } from "./layout/LayoutTree.tsx";
import { Tabs } from "./layout/Tabs.tsx";
import { ChannelList } from "./ui/ChannelList.tsx";
import { StatusBar } from "./ui/StatusBar.tsx";
import styles from "./App.module.css";

export function App({ session }: { session: Session }) {
  const status = useStatus(session);
  const teleop = teleopHooks(session);

  let content;
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
    content = (
      <Tabs manifest={status.manifest} store={session.store} teleop={teleop}>
        <LayoutTree manifest={status.manifest} store={session.store} teleop={teleop} />
        <ChannelList
          channels={status.manifest.channels}
          panels={status.manifest.panels}
          store={session.store}
        />
      </Tabs>
    );
  }

  return (
    <div className={styles.app}>
      <StatusBar status={status} />
      {/* A changed manifest remounts everything below the status bar. */}
      <main className={styles.main} key={status.epoch}>
        {content}
      </main>
    </div>
  );
}
