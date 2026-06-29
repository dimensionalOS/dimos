// @dimos/react — thin React bindings over @dimos/topics.
// High-rate topics: prefer useTopicRef (no re-render; read in a rAF loop) over
// useTopicLatest (re-renders per message).
import {
  createContext,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
  type MutableRefObject,
  type ReactNode,
} from "react";
import { connect, type DimosClient } from "@dimos/topics";
import type { MessageMeta, Status, TopicInfo, TopicStats } from "@dimos/topics";

interface DimosCtx {
  client: DimosClient | null;
  status: Status;
}
const Ctx = createContext<DimosCtx>({ client: null, status: "connecting" });

export function DimosProvider({ url, children }: { url: string; children: ReactNode }) {
  const [client, setClient] = useState<DimosClient | null>(null);
  const [status, setStatus] = useState<Status>("connecting");
  useEffect(() => {
    let alive = true;
    let c: DimosClient | undefined;
    connect({ url }).then((cl) => {
      if (!alive) {
        cl.close();
        return;
      }
      c = cl;
      setClient(cl);
      setStatus(cl.status);
      cl.onStatus(setStatus);
    });
    return () => {
      alive = false;
      c?.close();
    };
  }, [url]);
  return <Ctx.Provider value={{ client, status }}>{children}</Ctx.Provider>;
}

export const useDimos = () => useContext(Ctx);
export const useDimosClient = () => useContext(Ctx).client;
export const useStatus = () => useContext(Ctx).status;

/** Live list of discovered topics. */
export function useTopics(): TopicInfo[] {
  const client = useDimosClient();
  const [topics, setTopics] = useState<TopicInfo[]>(() => client?.listTopics() ?? []);
  useEffect(() => {
    if (!client) return;
    setTopics(client.listTopics());
    return client.onTopics(setTopics);
  }, [client]);
  return topics;
}

/** Latest message for a topic; re-renders on each delivery (use maxHz for high-rate). */
export function useTopicLatest<T = unknown>(
  topic: string | null,
  opts?: { maxHz?: number },
): { data?: T; meta?: MessageMeta } {
  const client = useDimosClient();
  const [state, setState] = useState<{ data?: T; meta?: MessageMeta }>({});
  const maxHz = opts?.maxHz;
  useEffect(() => {
    if (!client || !topic) return;
    const t = client.topic<T>(topic);
    if (maxHz) t.setRateLimit(maxHz);
    const sub = t.subscribeLatest((data, meta) => setState({ data, meta }));
    return () => sub.unsubscribe();
  }, [client, topic, maxHz]);
  return state;
}

/** A ref updated on every message WITHOUT re-rendering — read it in a rAF loop (canvas). */
export function useTopicRef<T = unknown>(
  topic: string | null,
): MutableRefObject<{ data?: T; meta?: MessageMeta }> {
  const client = useDimosClient();
  const ref = useRef<{ data?: T; meta?: MessageMeta }>({});
  useEffect(() => {
    if (!client || !topic) return;
    const sub = client.topic<T>(topic).subscribe((data, meta) => {
      ref.current = { data, meta };
    });
    return () => sub.unsubscribe();
  }, [client, topic]);
  return ref;
}

/** Live stats for a topic (polled). Keeps the topic subscribed so stats flow. */
export function useTopicStats(topic: string | null, pollMs = 500): TopicStats | null {
  const client = useDimosClient();
  const [stats, setStats] = useState<TopicStats | null>(null);
  useEffect(() => {
    if (!client || !topic) return;
    const t = client.topic(topic);
    const sub = t.subscribe(() => {}); // keep alive so stats accrue
    const id = setInterval(() => setStats(t.stats()), pollMs);
    return () => {
      clearInterval(id);
      sub.unsubscribe();
    };
  }, [client, topic, pollMs]);
  return stats;
}

/** Safe teleop helpers — gateway clamps + runs a TTL/deadman watchdog. */
export function useTeleop() {
  const client = useDimosClient();
  return useMemo(
    () => ({
      drive: (linearX: number, angularZ: number, ttlMs = 400) =>
        client?.teleop(linearX, angularZ, ttlMs),
      stop: () => client?.stop(),
    }),
    [client],
  );
}

export type { MessageMeta, Status, TopicInfo, TopicStats } from "@dimos/topics";
