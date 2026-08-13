// The React-facing edge of the session layer (the only React import under
// session/). Both hooks ride useSyncExternalStore; channel snapshots only
// change on the store's UI tick, so channel rate never sets render rate.

import { useCallback, useSyncExternalStore } from "react";
import type { ChannelSnapshot, ChannelStore, SessionStatus, StatusStore } from "./store.ts";

export function useStatus(store: StatusStore): SessionStatus {
  return useSyncExternalStore(store.subscribe, store.get);
}

export function useChannel(store: ChannelStore, ch: string): ChannelSnapshot {
  const subscribe = useCallback((cb: () => void) => store.subscribeUi(ch, cb), [store, ch]);
  const getSnapshot = useCallback(() => store.getUiSnapshot(ch), [store, ch]);
  return useSyncExternalStore(subscribe, getSnapshot);
}
