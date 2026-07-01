// The gateway address (host:port) as a runtime-editable app setting, shared between main.tsx (which
// builds the transport list from it) and the topbar input (App.tsx). Kept in its own module so
// main.tsx ↔ App.tsx don't form a circular import.
import { createContext, useContext } from "react";

export interface GatewayCtx {
  gateway: string; // host:port the transports connect to
  setGateway: (g: string) => void; // persist + reconnect
}

export const GatewayContext = createContext<GatewayCtx>({ gateway: "", setGateway: () => {} });
export const useGateway = () => useContext(GatewayContext);
