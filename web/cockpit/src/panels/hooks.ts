// Small React helpers shared by the cockpit panels.

import { useCallback, useSyncExternalStore } from "react";
import type { ChannelStore, Slot } from "@dimos/sdk";

const noop = (): void => {};

/** UI-tick slot of an optional channel (a panel role the manifest may not
 * bind): null both while unbound and before the first frame. */
export function useOptionalSlot(store: ChannelStore, ch: string | undefined): Slot | null {
  const subscribe = useCallback(
    (cb: () => void) => (ch === undefined ? noop : store.subscribeUi(ch, cb)),
    [store, ch],
  );
  const getSnapshot = useCallback(
    () => (ch === undefined ? null : store.getUiSnapshot(ch).slot),
    [store, ch],
  );
  return useSyncExternalStore(subscribe, getSnapshot);
}
