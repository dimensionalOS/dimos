// The React-facing edge of the SDK: the only React import in the package
// (React is a peer dependency; the root entry stays React-free). All hooks
// ride useSyncExternalStore; channel snapshots only change on the session's
// UI tick, so channel rate never sets render rate.

import { useCallback, useSyncExternalStore } from "react";
import type { Session } from "./session.ts";
import type { ChannelSnapshot, ChannelStore, SessionStatus, StatusStore } from "./store.ts";

/** Session status. The parameter is structural ({ status }) so any holder of
 * a StatusStore (e.g. the teleop hooks) works too. */
export function useStatus(session: { status: StatusStore }): SessionStatus {
  return useSyncExternalStore(session.status.subscribe, session.status.get);
}

/**
 * Refcounted channel subscription while mounted: acquires wire interest
 * through session.subscribe() and reads UI-tick snapshots. To observe a
 * store without creating wire interest, use useStoreChannel.
 */
export function useChannel(session: Session, ch: string): ChannelSnapshot {
  const subscribe = useCallback(
    (cb: () => void) => session.subscribe(ch, cb),
    [session, ch],
  );
  const getSnapshot = useCallback(() => session.store.getUiSnapshot(ch), [session, ch]);
  return useSyncExternalStore(subscribe, getSnapshot);
}

/** Read-only observation of one channel in a store: no wire interest, so an
 * unsubscribed channel renders as such instead of being subscribed. */
export function useStoreChannel(store: ChannelStore, ch: string): ChannelSnapshot {
  const subscribe = useCallback((cb: () => void) => store.subscribeUi(ch, cb), [store, ch]);
  const getSnapshot = useCallback(() => store.getUiSnapshot(ch), [store, ch]);
  return useSyncExternalStore(subscribe, getSnapshot);
}
