// Runtime-editable gateway address, shared between main.tsx and App.tsx — its own module to avoid a circular import.
import { createContext, useContext } from "react";

export interface GatewayCtx {
  gateway: string; // host:port the transports connect to
  setGateway: (g: string) => void; // persist + reconnect
}

export const GatewayContext = createContext<GatewayCtx>({ gateway: "", setGateway: () => {} });
export const useGateway = () => useContext(GatewayContext);

const GW_DEFAULT_PORT = "8080"; // keep in sync with main.tsx GW_PORT

/** People paste browser URLs — accept them: strip the scheme and anything past the host:port,
 *  default the port for a bare host. Bracketed IPv6 passes through (it carries a colon). */
export function normalizeGateway(raw: string): string {
  let v = raw.trim().replace(/^[a-z][a-z0-9+.-]*:\/\//i, "");
  v = v.split(/[/?#]/)[0].replace(/:$/, "");
  if (!v) return "";
  return v.includes(":") ? v : `${v}:${GW_DEFAULT_PORT}`;
}

// The gateway field's memory — an MRU of committed addresses, fed to a native <datalist>.
const RECENT_KEY = "dimos.gw.recent";
const RECENT_MAX = 8;

export function recentGateways(): string[] {
  try {
    const a = JSON.parse(localStorage.getItem(RECENT_KEY) ?? "[]");
    return Array.isArray(a) ? a.filter((x): x is string => typeof x === "string") : [];
  } catch {
    return [];
  }
}

export function pushRecentGateway(gw: string): void {
  const list = [gw, ...recentGateways().filter((g) => g !== gw)].slice(0, RECENT_MAX);
  try {
    localStorage.setItem(RECENT_KEY, JSON.stringify(list));
  } catch { /* quota — the memory is a convenience */ }
}
