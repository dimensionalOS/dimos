// React bindings over DimosBus. Three hooks are the whole public API:
//   useBus()    -> the bus instance (for publishing)
//   useTopics() -> { topics, status }, re-renders when discovery/status changes
//   useTopic(t) -> latest decoded message on topic t (or null)
import { createContext, type ReactNode, useContext, useEffect, useRef, useState } from "react";
import { type Decoded, DimosBus } from "./bus";

const BusCtx = createContext<DimosBus | null>(null);

export function BusProvider({ url, children }: { url: string; children: ReactNode }) {
  const ref = useRef<DimosBus | null>(null);
  if (!ref.current) ref.current = new DimosBus(url);
  return <BusCtx.Provider value={ref.current}>{children}</BusCtx.Provider>;
}

export function useBus(): DimosBus {
  const bus = useContext(BusCtx);
  if (!bus) throw new Error("useBus must be used inside <BusProvider>");
  return bus;
}

export function useTopics() {
  const bus = useBus();
  const [, force] = useState(0);
  useEffect(() => bus.onChange(() => force((n) => n + 1)), [bus]);
  return { topics: bus.topics, status: bus.status };
}

export function useTopic(topic: string | null): Decoded | null {
  const bus = useBus();
  const [msg, setMsg] = useState<Decoded | null>(null);
  useEffect(() => {
    setMsg(null);
    if (!topic) return;
    return bus.subscribe(topic, setMsg);
  }, [bus, topic]);
  return msg;
}
