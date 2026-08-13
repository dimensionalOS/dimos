import { useStatus } from "./session/hooks.ts";
import type { SessionHandle } from "./session/session.ts";
import { ChannelList } from "./ui/ChannelList.tsx";
import { StatusBar } from "./ui/StatusBar.tsx";
import styles from "./App.module.css";

export function App({ session }: { session: SessionHandle }) {
  const status = useStatus(session.status);

  let content;
  if (status.transport.phase === "failed") {
    content = <p className={styles.notice}>Connection failed: {status.transport.reason}</p>;
  } else if (status.robotCount > 1) {
    content = (
      <p className={styles.notice}>
        {status.robotCount} robots connected; the robot picker arrives in a later release.
      </p>
    );
  } else if (status.channels.length === 0) {
    content = <p className={styles.notice}>Waiting for a robot to register...</p>;
  } else {
    content = <ChannelList channels={status.channels} store={session.channels} />;
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
