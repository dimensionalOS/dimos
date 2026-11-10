// What keeps the view current: the gateway's zenoh events, and the few timers config.yaml's `blueprint-view:` sets.
import { useZenoh } from "dimos-react";
import { useEffect, useState } from "react";
import { getJson } from "./api.ts";

/** GET config's blueprintView: the timers for what no event tells (Desktop's rates, everything while zenoh is down). */
export type ViewConfig = { ratesPollS: number; pollWithoutEventsS: number; logMaxRecords: number };

/** The view's config, read again on its config event and after a reconnect; null until the first answer. */
export function useViewConfig(): ViewConfig | null {
  const zenoh = useZenoh();
  const [config, setConfig] = useState<ViewConfig | null>(null);
  useEffect(() => {
    const read = () =>
      getJson<{ blueprintView: ViewConfig }>("config").then(
        (answer) => setConfig(answer.blueprintView),
        () => {},
      );
    read();
    if (!zenoh) {
      return;
    }
    const offConfig = zenoh.subscribeDimos("config/blueprintView", read);
    const offReconnect = zenoh.onReconnect(read);
    return () => {
      offConfig();
      offReconnect();
    };
  }, [zenoh]);
  return config;
}

/** `run` one at a time: asked again while one is out, it runs once more after that one, never piling up. */
export function oneInFlight(run: () => Promise<unknown>): () => void {
  let running = false;
  let again = false;
  const go = async () => {
    if (running) {
      again = true;
      return;
    }
    running = true;
    try {
      do {
        again = false;
        await run().catch(() => {});
      } while (again);
    } finally {
      running = false;
    }
  };
  return go;
}

/** Calls `run` every `seconds` while that's a number (null: an event tells instead). */
export function useEvery(run: () => void, seconds: number | null | undefined) {
  useEffect(() => {
    if (seconds == null) {
      return;
    }
    const timer = setInterval(run, seconds * 1000);
    return () => clearInterval(timer);
  }, [run, seconds]);
}
