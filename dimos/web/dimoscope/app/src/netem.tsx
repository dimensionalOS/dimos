// App-global network-condition state (server-side netem, GET/POST /netem — gateway/netem.py,
// opt-in via NETEM_CTL=1 on a Linux gateway). One provider feeds the topbar select, the bench
// sweep chips, and the bench runner, so no two fetchers can disagree about what's shaped.
import {
  createContext,
  type ReactNode,
  useCallback,
  useContext,
  useEffect,
  useRef,
  useState,
} from "react";
import { useGateway } from "./gateway";

export interface NetemChip {
  id: string;
  label: string;
  desc: string;
}
export interface NetemState {
  enabled: boolean;
  active: string;
  healsInS: number;
  profiles: NetemChip[];
  outages: NetemChip[];
}

export async function getNetem(httpBase: string): Promise<NetemState | null> {
  try {
    return await (await fetch(`${httpBase}/netem`)).json();
  } catch {
    return null; // older gateway without /netem, or unreachable
  }
}

/** POST a profile/outage id; the response body IS the fresh state — no follow-up GET
 *  (/netem rides the shaped port, every extra round-trip crawls under `disaster`). */
export async function postNetem(httpBase: string, id: string): Promise<NetemState> {
  const res = await fetch(`${httpBase}/netem`, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ profile: id }),
  });
  const body = await res.json();
  if (!res.ok) throw new Error(String(body.error ?? res.status));
  return body;
}

interface NetemCtx {
  /** null = endpoint absent/unreachable; `enabled: false` = present but not opted in. */
  netem: NetemState | null;
  /** First fetch has settled — before this, `netem: null` may just mean "still asking". */
  ready: boolean;
  busy: boolean;
  msg?: string;
  refresh: () => Promise<NetemState | null>;
  /** POST + share the fresh state; resolves null (and sets `msg`) on failure. */
  apply: (id: string) => Promise<NetemState | null>;
}
const Ctx = createContext<NetemCtx>({
  netem: null,
  ready: false,
  busy: false,
  refresh: () => Promise.resolve(null),
  apply: () => Promise.resolve(null),
});
export const useNetem = () => useContext(Ctx);

export function NetemProvider({ children }: { children: ReactNode }) {
  const { gateway } = useGateway();
  const httpBase = `${location.protocol}//${gateway}`;
  const [netem, setNetem] = useState<NetemState | null>(null);
  const [ready, setReady] = useState(false);
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState<string>();
  // Bumped by every successful POST — a slow GET that started before it must not
  // overwrite the fresher state (the matrix runner flips profiles while ticks/focus refresh).
  const epoch = useRef(0);

  const refresh = useCallback(async () => {
    const started = epoch.current;
    const st = await getNetem(httpBase);
    if (epoch.current === started) setNetem(st);
    setReady(true);
    return st;
  }, [httpBase]);

  const apply = useCallback(async (id: string) => {
    setBusy(true);
    setMsg(undefined);
    try {
      const st = await postNetem(httpBase, id);
      epoch.current++;
      setNetem(st);
      return st;
    } catch (e) {
      setMsg(`✗ ${(e as Error).message}`);
      return null;
    } finally {
      setBusy(false);
    }
  }, [httpBase]);

  // Mount / gateway change + window focus keep the view honest against CLI-side changes;
  // a slow tick only while shaped catches the self-heal without polling an idle gateway.
  useEffect(() => {
    refresh();
    const onFocus = () => refresh();
    globalThis.addEventListener("focus", onFocus);
    return () => globalThis.removeEventListener("focus", onFocus);
  }, [refresh]);
  const shaped = !!netem?.enabled && netem.active !== "clean";
  useEffect(() => {
    if (!shaped) return;
    const iv = setInterval(refresh, 60_000);
    return () => clearInterval(iv);
  }, [shaped, refresh]);

  return <Ctx.Provider value={{ netem, ready, busy, msg, refresh, apply }}>{children}</Ctx.Provider>;
}
